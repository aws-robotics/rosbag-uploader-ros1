import * as core from '@actions/core';
import * as exec from '@actions/exec';
import * as github from '@actions/github';

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

installPackages();