import * as core from '@actions/core';
import * as exec from '@actions/exec';
import * as github from '@actions/github';
import { ExecOptions } from '@actions/exec/lib/interfaces';

async function build() {
  try {

    const ROS_DISTRO = "kinetic" 
    
    const execOptions: ExecOptions = {
      cwd: core.getInput('package-path')
    };

    await exec.exec("rosdep", ["install", "--from-paths", ".", "--ignore-src", "-r", "-y", "--rosdistro", ROS_DISTRO], execOptions);
    await exec.exec("colcon", ["build"], execOptions);

  } catch (error) {
    core.setFailed(error.message);
  }
}

async function run() {
  await build();
}

run();