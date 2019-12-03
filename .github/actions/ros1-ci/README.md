# ROS1 CI Github Action

This action will build, test, and generate code coverage for your ROS1 package.
It must be run in an environment that has all core ROS1 dependencies already installed for the ROS1 distro you are using (Kinetic, Melodic etc). 

You can either use the action [setup-ros1] to create this environment or use the [ros-core docker container], see usage section to see how to use this container. 

## Usage

Using a [ros-core docker container] docker container:

```
jobs:
  build-kinetic:
    runs-on: ubuntu-latest
    name: Build Kinetic
    container:
      image: ros:kinetic-ros-core
    steps:
    - name: Build
      uses: actions/ros1-ci@v1
      with:
        coverage: true
        language: cpp
        packages-to-test: "s3_common s3_file_uploader"
        ros-distro: kinetic
        test: true
        workspace-dir: ./
```

Using [setup-ros1]:

```
jobs:
  build-kinetic:
    runs-on: ubuntu-latest
    name: Build Kinetic
    steps:
    - name: Setup
      uses: actions/setup-ros1@v1
    - name: Build
      uses: actions/ros1-ci@v1
      with:
        coverage: true
        language: cpp
        packages-to-test: "s3_common s3_file_uploader"
        ros-distro: kinetic
        test: true
        workspace-dir: ./
```

## Inputs

### `coverage`

Boolean - Should code coverage be run. **Default: true**

### `language`

**Required** The language your ROS package is in `[cpp|python]`

### `packages-to-test`

**Required** Space separated string of packages you want to run tests for

### `ros-distro`

**Required** Distribution of ROS you are using `[kinetic|melodic]`

### `test`

Boolean - Should tests be run. **Default: true** 

### `workspace-dir`

Path to the workspace folder of your package. **Default: ./**

[setup-ros1]: https://github.com/ros-tooling/setup-ros1
[ros-core docker container]: https://hub.docker.com/_/ros/