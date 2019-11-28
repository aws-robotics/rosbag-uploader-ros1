import * as core from '@actions/core';
import * as exec from '@actions/exec';
import * as github from '@actions/github';
import { ExecOptions } from '@actions/exec/lib/interfaces';
import * as path from 'path';

const COVERAGE_ARTIFACT_NAME = "coverage.tar"

function getExecOptions() {
  const workspaceDir = core.getInput('workspace-dir');
  const rosDistro = core.getInput('ros-distro');
  const execOptions: ExecOptions = {
    cwd: workspaceDir,
    env: Object.assign({}, process.env, {
      CMAKE_PREFIX_PATH: `/opt/ros/${rosDistro}`,
      ROS_DISTRO: rosDistro,
      ROS_ETC_DIR: `/opt/ros/${rosDistro}/etc/ros`,
      ROS_PACKAGE_PATH: `/opt/ros/${rosDistro}/share`,
      ROS_PYTHON_VERSION: "2",
      ROS_ROOT: `/opt/ros/${rosDistro}/share/ros`,
      ROS_VERSION: "1"
    })
  };
  return execOptions
}

async function setup() {
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
    const rosDistro = core.getInput('ros-distro');
    await exec.exec("rosdep", ["install", "--from-paths", ".", "--ignore-src", "-r", "-y", "--rosdistro", rosDistro], getExecOptions());

    const colconCmakeArgs = [
      "--cmake-args", 
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON", 
      "-DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage'", 
      "-DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'"
    ]
    await exec.exec("colcon", ["build"].concat(colconCmakeArgs), getExecOptions());
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function test() {
  try {
    if (!core.getInput('test')) {
      console.log("Skipping testing as test flag is false");
      return;
    }
    const workspaceDir = core.getInput('workspace-dir');
    const packagesToTest = core.getInput('packages-to-test');

    if (packagesToTest.length) {
      const colconCmakeTestArgs = [
        "--packages-select",
      ].concat(
        packagesToTest.split(" "),
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
    core.addPath(path.join(workspaceDir, "install", "bin"))

    await exec.exec("colcon", ["test"], getExecOptions());
    await exec.exec("colcon", ["test-result", "--all", "--verbose"], getExecOptions());

  } catch (error) {
    core.setFailed(error.message);
  }
}

async function coverage() {
  try {
    if (!core.getInput('coverage')) {
      console.log("Skipping producing code coverage as coverage flag is false")
      return;
    } 

    const packageLanguage = core.getInput('language');
    const packagesToTest = core.getInput('packages-to-test');
    const workspaceDir = core.getInput('workspace-dir');
    
    const execOptions = getExecOptions();

    if (packageLanguage == "cpp") {
      await exec.exec("lcov", ["--capture", "--directory", ".", "--output-file", "coverage.info"], execOptions);
      await exec.exec("lcov", ["--remove", "coverage.info", '/usr/*', '--output-file', 'coverage.info'], execOptions);
      await exec.exec("lcov", ["--list", "coverage.info"], execOptions);
    } 
    else if (packageLanguage == "python") {
      const allPackages = packagesToTest.split(" ")
      allPackages.forEach(async (packageName) => {
        const packageExecOptions = getExecOptions();
        const workingDir = path.join(workspaceDir, 'build', packageName);
        packageExecOptions.cwd = workingDir;

        await exec.exec("coverage", ["xml"], packageExecOptions);
        await exec.exec("mv", ["coverage.xml", `coverage-${packageName}.info`], packageExecOptions);
      });
    }

    // Create coverage artifact for exporting
    await exec.exec("tar", ["cvf", COVERAGE_ARTIFACT_NAME, "coverage*.info"], execOptions)
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function run() {
  await setup();
  await build();
  await test();
  await coverage();
}

run();