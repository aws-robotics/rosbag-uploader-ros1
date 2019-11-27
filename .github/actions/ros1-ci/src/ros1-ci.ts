import * as core from '@actions/core';
import * as exec from '@actions/exec';
import * as github from '@actions/github';
import { ExecOptions } from '@actions/exec/lib/interfaces';
import * as path from 'path';


async function installPackages() {
  try {
    const aptPackages = [
      "lcov", 
      "python-pip", 
      "python3-pip", 
      "python-rosinstall", 
      "libgtest-dev", 
      "cmake", 
      "python3-colcon-common-extensions"
    ];

    const python2Packages = [
      "coverage"
    ];

    const python3Packages = [
      "setuptools"
    ];

    await exec.exec("sudo", ["apt-get", "update"]);
    await exec.exec("sudo", ["apt-get", "install", "-y"].concat(aptPackages));
    await exec.exec("sudo", ["pip", "install", "-U"].concat(python2Packages));
    await exec.exec("sudo", ["pip3", "install", "-U"].concat(python3Packages));

    await exec.exec("rosdep", ["update"]);

  } catch (error) {
    core.setFailed(error.message);
  }
}


async function build() {
  try {

    const WORKSPACE_DIR = core.getInput('workspace-dir');
    const ROS_DISTRO = core.getInput('ros-distro');
    const PACKAGES_TO_TEST = core.getInput('packages-to-test');
    
    function getExecOptions() {
      const execOptions: ExecOptions = {
        cwd: WORKSPACE_DIR,
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
      return execOptions
    }

    await exec.exec("rosdep", ["install", "--from-paths", ".", "--ignore-src", "-r", "-y", "--rosdistro", ROS_DISTRO], getExecOptions());

    const colconCmakeArgs = [
      "--cmake-args", 
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON", 
      "-DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage'", 
      "-DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'"
    ]
    await exec.exec("colcon", ["build"].concat(colconCmakeArgs), getExecOptions());

    if (PACKAGES_TO_TEST.length) {
      const colconCmakeTestArgs = [
        "--packages-select",
      ].concat(
        PACKAGES_TO_TEST.split(" "),
        [
          "--cmake-target",
          "tests"
        ]
      )
      await exec.exec("colcon", ["build"].concat(colconCmakeTestArgs), getExecOptions());
    }

    // Add the future install bin directory to PATH.
    // This enables cmake find_package to find packages installed in the
    // colcon install directory, even if local_setup.sh has not been sourced.
    //
    // From the find_package doc:
    // https://cmake.org/cmake/help/latest/command/find_package.html
    //   5. Search the standard system environment variables.
    //   Path entries ending in /bin or /sbin are automatically converted to
    //   their parent directories:
    //   PATH
    core.addPath(path.join(WORKSPACE_DIR, "install", "bin"))

    await exec.exec("colcon", ["test"], getExecOptions());
    await exec.exec("colcon", ["test-result", "--all", "--verbose"], getExecOptions());

  } catch (error) {
    core.setFailed(error.message);
  }
}

async function run() {
  await installPackages();
  await build();
}

run();