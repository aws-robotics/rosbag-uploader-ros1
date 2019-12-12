import * as core from '@actions/core';
import * as exec from '@actions/exec';

async function postBuildStatus() {
  try {
    const buildStatus = core.getInput('status', { required: true });
    const validBuildStatusCheck = new RegExp('(success|failure)');
    if (!buildStatus.match(validBuildStatusCheck)) {
      throw new Error(`Invalid build status ${buildStatus} passed to cw-build-status`);
    }

    console.log("Received build status: ", buildStatus);
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function run() {
  await postBuildStatus();
}

run();
