import * as path from 'path';
import * as core from '@actions/core';
import * as exec from '@actions/exec';

const ACTION_DIRECTORY = path.resolve(__dirname, "..");

async function setup() {
  try {
    const aptPackages = [
      "clang-tools-6.0",
      "clang-tidy-6.0"
    ];

    await exec.exec("sudo", ["apt-get", "update"]);
    await exec.exec("sudo", ["apt-get", "install", "-y"].concat(aptPackages));

  } catch (error) {
    core.setFailed(error.message);
  }
}

async function runClangTidyScript() {
  try {
    const workspaceDir = core.getInput('workspace-dir');

    const execOptions = {
      cwd: workspaceDir
    }

    const clangTidyVerifyPath = path.join(ACTION_DIRECTORY, "clang-tidy-verify.py");

    await exec.exec(clangTidyVerifyPath, [], execOptions);

  } catch (error) {
    core.setFailed(error.message);
  } 
}

async function run() {
  await setup();
  await runClangTidyScript();
}

run();
