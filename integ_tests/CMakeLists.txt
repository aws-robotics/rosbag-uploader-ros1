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
        tests/file_helpers.py
        tests/s3_file_uploader_test_base.py
        tests/test_s3_file_uploader_general.py
        tests/test_s3_file_uploader_wrong_region.py
        tests/s3_client.py
        scripts/setup_test_bucket.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY configurations launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
