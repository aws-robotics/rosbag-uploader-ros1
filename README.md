## Rosbag Cloud Recorders

This repository contains ROS nodes for recording rosbags and uploading them to Amazon S3.

There are three nodes under this repository: the `duration_recorder`, the `rolling_recorder`, and the `s3_file_uploader`. See the READMEs under the `rosbag_cloud_recorders` and `s3_uploader` directories for more details.
- `duration_recorder` - This node provides an action interface to record rosbags for a specified duration. Once that duration is complete the rosbag files are uploaded to S3.
- `rolling_recorder` - This node provides an action interface for uploading the past *x* minutes of rosbag files.
- `s3_file_uploader` - This node is in the `s3_file_uploader` package and provides an action interface for uploading a set of files to S3.

More details on the `duration_recorder` and `rolling_recorder` can be found in the `rosbag_cloud_recorders` README. More details on the `s3_file_uploader` can be found in the `s3_file_uploader` README.

**Amazon S3 Summary**: Amazon Simple Storage Service (Amazon S3) is an object storage service that offers industry-leading scalability, data availability, security, and performance. This means customers of all sizes and industries can use it to store and protect any amount of data for a range of use cases, such as websites, mobile applications, backup and restore, archive, enterprise applications, IoT devices, and big data analytics. Amazon S3 provides easy-to-use management features so you can organize your data and configure finely-tuned access controls to meet your specific business, organizational, and compliance requirements. Amazon S3 is designed for 99.999999999% (11 9's) of durability, and stores data for millions of applications for companies all around the world.

### Build status
* GitHub Action Status
     * master: ![Build & Test](https://github.com/aws-robotics/rosbag-uploader-ros1/workflows/Build%20&%20Test/badge.svg?branch=master&event=schedule)
     * release-latest: ![Build & Test release-latest](https://github.com/aws-robotics/rosbag-uploader-ros1//workflows/Build%20&%20Test%20release-latest/badge.svg?event=schedule)



[![Actions Status](https://github.com/aws-robotics/rosbag-uploader-ros1/workflows/build-test/badge.svg)](https://github.com/aws-robotics/rosbag-uploader-ros1/actions)


## License

The source code is released under [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com


## Installation

### Building from Source

To build from source you'll need to create a new workspace, clone and checkout the `release-latest` branch of this repository, install all the dependencies, and compile. If you need the latest development features you can clone from the `master` branch instead of the latest release branch. While we guarantee the release branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development.

- Install build tool: please refer to `colcon` [installation guide](https://colcon.readthedocs.io/en/released/user/installation.html)

- Create a ROS workspace and a source directory

        mkdir -p ~/ros-workspace/src

- Clone the package into the source directory . 

        cd ~/ros-workspace/src
        git clone https://github.com/aws-robotics/rosbag-uploader-ros1.git

- Install dependencies

        cd ~/ros-workspace 
        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y

_Note: If building the master branch instead of a release branch you may need to also checkout and build the master branches of the packages this package depends on._

- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library Path

        source ~/ros-workspace/install/setup.bash

### Running Tests

- Build with

        colcon build --cmake-targets tests

- Then run tests

        colcon test
        colcon test-result --all

### Running

Instructions for running the nodes in this repository can be found in the READMEs for `rosbag_cloud_recorders` and `s3_file_uploader` packages.


## Bugs & Feature Requests

Please contact the team directly if you would like to request a feature.

Please report bugs in [Issue Tracker].


[Amazon Web Services (AWS)]: https://aws.amazon.com/
[Apache 2.0]: https://aws.amazon.com/apache-2-0/
[Issue Tracker]: https://github.com/aws-robotics/rosbag-uploader-ros1/issues
[ROS]: http://www.ros.org
