import * as path from 'path';
import * as core from '@actions/core';
import * as exec from '@actions/exec';
import * as github from '@actions/github';
import { ExecOptions } from '@actions/exec/lib/interfaces';

const ROS_ENV_VARIABLES: any = {};
const INSTALL_ENV_VARS: any = {};
const ROS_DISTRO = core.getInput('ros-distro', {required: true});

// This github action will be in its own repo prior to public launch.
// The following method therefore prevents the integration tests from
// being run on PRs & on forks.
// AWS-Robotics integration tests tend to interact with actual AWS Services
// and therefore currently are intended to be run only as a part of CRON jobs
// on the master branch
function shouldSkip():boolean {
  const context = github.context;
  const isNotPull = context.eventName !== 'pull_request';
  const isMaster = context.ref === 'refs/heads/master';
  const isAwsRobotics = context.repo.owner === 'aws-robotics';
  const shouldSkip = !(isNotPull && isAwsRobotics && isMaster);
  return shouldSkip;
}

async function loadEnvVariables(envVars:any, command:string) {
  const options = {
    listeners: {
      stdout: (data: Buffer) => {
        const lines = data.toString().split("\n");
        lines.forEach(line => {
          if (line.trim().length === 0) return;
          const contents = line.trim().split("=");
          envVars[contents[0]] = contents.slice(1).join("=");
        });
      }
    }
  };
  
  await exec.exec("bash", ["-c", command], options);
}

async function setup() {
  try {
    const workspaceDir = core.getInput('workspace-dir');
    await loadEnvVariables(ROS_ENV_VARIABLES, `source /opt/ros/${ROS_DISTRO}/setup.bash && printenv`);
    await loadEnvVariables(INSTALL_ENV_VARS, `source ${workspaceDir}/install/setup.bash && printenv`);  
  } catch(error) {
    core.setFailed(error.message);
  }
}

async function rostest() {
  try {  
    const integTestPkgName = core.getInput('integ-test-package-name', {required: true});
    const integTestLaunchFileName = core.getInput('integ-test-launch-file-name', {required: true});
    const workspaceDir = core.getInput('workspace-dir');
    core.addPath(path.join(workspaceDir, "install", "bin"));
    const execOptions: ExecOptions = {
      cwd: workspaceDir,
      env: Object.assign({}, process.env, ROS_ENV_VARIABLES, INSTALL_ENV_VARS)
    };
    await exec.exec(`/opt/ros/${ROS_DISTRO}/bin/rostest`, [integTestPkgName, integTestLaunchFileName], execOptions)
  } catch (error) {
    core.setFailed(error.message);
  }
}
async function run() {
  if (shouldSkip()) {
    core.warning('Skipping the integration tests');
    return;
  }
  await setup();
  await rostest();
}

run()
