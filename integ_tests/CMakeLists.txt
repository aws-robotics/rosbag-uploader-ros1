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
        tests/duration_recorder_test_base.py
        tests/file_helpers.py
        tests/rolling_recorder_test_base.py
        tests/test_duration_recorder_general.py
        tests/test_periodic_file_deleter.py
        tests/test_s3_file_uploader_general.py
        tests/test_s3_file_uploader_wrong_region.py
        tests/test_rolling_recorder_custom_topic.py
        tests/test_rolling_recorder_default_topics.py
        tests/test_rolling_recorder_upload.py
        tests/s3_client.py
        tests/s3_file_uploader_test_base.py
        scripts/setup_test_bucket.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY configurations launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
