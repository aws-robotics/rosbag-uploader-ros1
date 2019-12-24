cmake_minimum_required(VERSION 2.8.3)
project(rosbag_uploader_ros1_integration_tests)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(
        LIBRARIES rosbag_uploader_ros1_integration_tests
)
#############
## Install ##
#############
install(PROGRAMS
        tests/test_rosbag_uploader_ros1.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY configurations launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
