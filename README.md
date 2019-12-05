## Rosbag Uploader Ros1

ROS packages for uploading rosbags to AWS cloud services.

## License

This library is licensed under the Apache 2.0 License. 

## Installation
### Building from Source
- Create a ROS workspace and a source directory

        mkdir -p ~/ros-workspace/src

- Clone the package into the source directory . 

        cd ~/ros-workspace/src
        git clone
- Install dependencies

        cd ~/ros-workspace 
        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y
- Build the packages

        cd ~/ros-workspace && colcon build

### Running Tests
- Build with

        colcon build --cmake-targets tests

- Then run tests

    colcon test

### Simple Upload File Test
```
 rostopic pub  /s3_file_uploader/UploadFiles/goal   file_uploader_msgs/UploadFilesActionGoal "{goal: { files:['/tmp/test_file.txt'], upload_location: 'foo/bar' } }"
```