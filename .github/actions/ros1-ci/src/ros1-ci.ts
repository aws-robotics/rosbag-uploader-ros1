import * as core from '@actions/core';
import * as exec from '@actions/exec';
import * as github from '@actions/github';

async function build() {
  try {
    
    await exec.exec("cd", [core.getInput('package-path')]);
    await exec.exec("rosdep", ["install", "--from-paths", ".", "--ignore-src", "-r", "-y"]);
    await exec.exec("colcon", ["build"]);

  } catch (error) {
    core.setFailed(error.message);
  }

}

build();