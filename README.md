
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

