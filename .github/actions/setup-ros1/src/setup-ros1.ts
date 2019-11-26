import * as core from '@actions/core';
import * as exec from '@actions/exec';
import * as github from '@actions/github';

async function installPackages() {
  try {
    const aptPackages = [
      "lcov", 
      "python-pip", 
      "python-pip3", 
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
    ]

    await exec.exec("bash", ["apt-get", "install"].concat(aptPackages))
    await exec.exec("bash", ["pip", "install"].concat(python2Packages))
    await exec.exec("bash", ["pip3", "install"].concat(python3Packages))

  } catch (error) {
    core.setFailed(error.message);
  }
}

installPackages();