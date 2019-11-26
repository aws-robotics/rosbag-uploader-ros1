#!/bin/bash

export CATKIN_TEST_COVERAGE=1

ROS_DISTRO=$1
PACKAGE_PATH=$2

rosdep update

cd $PACKAGE_PATH
[ -e ".rosinstall" ] && rosws update
rosdep install --from-paths . --ignore-src --rosdistro "${ROS_DISTRO}" -r -y

. /opt/ros/${ROS_DISTRO}/setup.sh

colcon build