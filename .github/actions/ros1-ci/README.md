# ROS1 CI Github Action

This action will build, test, and generate code coverage for your ROS1 package. 

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

## Example usage

uses: ros-tooling/ros1-ci@v1
with:
  coverage: true
  language: 'python'
  packages-to-test: 'my-cool-package another-package'
  ros-distro: 'kinetic'
  test: true
  workspace-dir: './packages/'