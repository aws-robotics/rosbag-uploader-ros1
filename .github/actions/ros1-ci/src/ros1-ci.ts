import * as path from 'path';
import * as core from '@actions/core';
import * as exec from '@actions/exec';
import { ExecOptions } from '@actions/exec/lib/interfaces';

const COVERAGE_FOLDER_NAME = "coverage";
const ROS_ENV_VARIABLES: any = {};
const ROS_DISTRO = core.getInput('ros-distro', {required: true});

async function loadROSEnvVariables() {
  const options = {
    listeners: {
      stdout: (data: Buffer) => {
        const lines = data.toString().split("\n");
        lines.forEach(line => {
          if (line.trim().length === 0) return;
          const contents = line.trim().split("=");
          ROS_ENV_VARIABLES[contents[0]] = contents.slice(1).join("=");
        });
      }
    }
  };

  await exec.exec("bash", [
  "-c",
  `source /opt/ros/${ROS_DISTRO}/setup.bash && printenv`
  ], options)
}

function getExecOptions(): ExecOptions {
  const workspaceDir = core.getInput('workspace-dir');
  const execOptions: ExecOptions = {
    cwd: workspaceDir,
    env: Object.assign({}, process.env, ROS_ENV_VARIABLES)
  };
  return execOptions
}

async function setup() {
  try {
    const aptPackages = [
      "cmake",
      "lcov",
      "libgtest-dev",
      "python-pip",
      "python-rosinstall",
      "python3-colcon-common-extensions",
      "python3-pip"
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

    await loadROSEnvVariables();

    // Download dependencies not in apt if .rosinstall exists
    await exec.exec("rosws", ["update", "2>/dev/null"], getExecOptions());
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function build() {
  try {
    await exec.exec("rosdep", ["install", "--from-paths", ".", "--ignore-src", "-r", "-y", "--rosdistro", ROS_DISTRO], getExecOptions());

    const packagesToSkipTests = core.getInput('packages-to-skip-tests');
    let colconUpToCmakeArgs: any = [];
    if (packagesToSkipTests.length) {
      colconUpToCmakeArgs = ["--packages-up-to", ].concat(packagesToSkipTests.split(" "));
    }
    await exec.exec("colcon", ["build"].concat(colconUpToCmakeArgs), getExecOptions());

    let colconCmakeArgs: any = []
    if (core.getInput('coverage')) {
      if (packagesToSkipTests.length) {
        colconCmakeArgs = [
          "--packages-skip",
        ].concat(packagesToSkipTests.split(" "));
      }
      colconCmakeArgs = colconCmakeArgs.concat([
        "--cmake-args",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "-DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage'",
        "-DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'"
      ]);
      await exec.exec("colcon", ["build"].concat(colconCmakeArgs), getExecOptions());
    }

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
    const packagesToSkipTests = core.getInput('packages-to-skip-tests');

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

    let colconArgs: any = []
    if (packagesToSkipTests.length) {
      colconArgs = [
        "--packages-skip",
      ].concat(packagesToSkipTests.split(" "));
    }
    await exec.exec("colcon", ["test"].concat(colconArgs), getExecOptions());
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
      await exec.exec("lcov", ["--remove", "coverage.info", '/usr/*', '/opt/ros/*', '*/test/*', '--output-file', 'coverage.info'], execOptions);
      await exec.exec("lcov", ["--list", "coverage.info"], execOptions);
      await exec.exec("mkdir", ["-p", COVERAGE_FOLDER_NAME])
      await exec.exec("cp", ["coverage.info", COVERAGE_FOLDER_NAME], execOptions)
    } 
    else if (packageLanguage == "python") {
      const allPackages = packagesToTest.split(RegExp('\\s'));
      await Promise.all(allPackages.map(async (packageName) => {
        const packageExecOptions = getExecOptions();
        const workingDir = path.join(workspaceDir, 'build', packageName);
        packageExecOptions.cwd = workingDir;
        const coverageFileName = `coverage-${packageName}.info`

        await exec.exec("coverage", ["xml"], packageExecOptions);
        await exec.exec("mv", ["coverage.xml", path.join(workspaceDir, COVERAGE_FOLDER_NAME, coverageFileName)], packageExecOptions);
        return coverageFileName;
      }));
    }

    // Create coverage artifact for exporting
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
