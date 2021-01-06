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
  const isAwsRobotics = context.repo.owner === 'aws-robotics';
  const shouldSkip = !isAwsRobotics;
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
    // NOTE: This is a kludge to just get newer versions of the library for this integration test.
    // We need SigV4 just for this test for internal reasons,
    // but the APT versions on Ubuntu Xenial are not new enough to fully support it
    if (ROS_DISTRO == "kinetic") {
      // Remove the APT version of boto
      await exec.exec("sudo", ["apt-get", "remove", "--yes", "python-boto3", "python-botocore"]);
      // Make sure we have pip
      await exec.exec("sudo", ["apt-get", "install", "--yes", "--no-install-recommends", "--quiet", "python-pip"]);
      // Install the newer version of the library
      await exec.exec("python", ["-m", "pip", "install", "boto3"])
    }


    await loadEnvVariables(ROS_ENV_VARIABLES, `source /opt/ros/${ROS_DISTRO}/setup.bash && printenv`);
    await loadEnvVariables(INSTALL_ENV_VARS, `source ${workspaceDir}/install/setup.bash && printenv`);
  } catch(error) {
    core.setFailed(error.message);
  }
}

async function rostest() {
  try {
    const integTestPkgName = core.getInput('integ-test-package-name', {required: true});
    const integTestLaunchFilesString = core.getInput('integ-test-launch-files', {required: true});
    const integTestLaunchFiles = integTestLaunchFilesString.split("\n");
    const workspaceDir = core.getInput('workspace-dir');
    core.addPath(path.join(workspaceDir, "install", "bin"));
    const execOptions: ExecOptions = {
      cwd: workspaceDir,
      env: Object.assign({}, process.env, ROS_ENV_VARIABLES, INSTALL_ENV_VARS)
    };

    for (let i = 0; i < integTestLaunchFiles.length; i++) {
      let integTestLaunchFileName = integTestLaunchFiles[i];
      if (integTestLaunchFileName.trim().length > 0) {
        await exec.exec(`/opt/ros/${ROS_DISTRO}/bin/rostest`, [integTestPkgName, integTestLaunchFileName], execOptions)
      }
    }
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
