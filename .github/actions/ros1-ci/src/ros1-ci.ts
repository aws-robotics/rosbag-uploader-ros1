import * as core from '@actions/core';
import * as exec from '@actions/exec';
import * as github from '@actions/github';
import { ExecOptions } from '@actions/exec/lib/interfaces';

async function build() {
  try {

    const ROS_DISTRO = core.getInput('ros-distro');
    
    const execOptions: ExecOptions = {
      cwd: core.getInput('workspace-dir'),
      env: Object.assign({}, process.env, {
        CMAKE_PREFIX_PATH: `/opt/ros/${ROS_DISTRO}`,
        ROS_DISTRO: ROS_DISTRO,
        ROS_ETC_DIR: `/opt/ros/${ROS_DISTRO}/etc/ros`,
        ROS_PACKAGE_PATH: `/opt/ros/${ROS_DISTRO}/share`,
        ROS_PYTHON_VERSION: "2",
        ROS_ROOT: `/opt/ros/${ROS_DISTRO}/share/ros`,
        ROS_VERSION: "1"
      })
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