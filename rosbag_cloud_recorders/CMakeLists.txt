cmake_minimum_required(VERSION 2.8.3)
project(rosbag_cloud_recorders)

set(CMAKE_CXX_STANDARD 14)

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
# Removed -Werror=pedantic until ros.h is fixed
set(FLAGS -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

if(NOT DEFINED CXX_DISABLE_WERROR)
  set(WARNING_CXX_FLAGS "-Werror ${WARNING_CXX_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    aws_common
    aws_ros1_common
    file_uploader_msgs
    recorder_msgs
    roscpp
    rosbag
    roslint
)

set(ROSBAG_CLOUD_RECORDER_LIB_TARGET ${PROJECT_NAME}_lib)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    CATKIN_DEPENDS actionlib actionlib_msgs recorder_msgs roscpp rosbag file_uploader_msgs
)

###########
## Build ##
###########
roslint_cpp(src/rolling_recorder/rolling_recorder.cpp
            src/rolling_recorder/main.cpp
            test/rolling_recorder_test.cpp
            test/rolling_recorder_action_server_handler_test.cpp
            include/rolling_recorder/rolling_recorder.h
            include/rolling_recorder/rolling_recorder_action_server_handler.h
            test/duration_recorder_action_server_handler_test.cpp
            include/rolling_recorder/rolling_recorder.h
            include/duration_recorder/duration_recorder_action_server_handler.h
            include/utils/rosbag_recorder.h)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${aws_common_INCLUDE_DIR}
    ${aws_ros1_common_INCLUDE_DIR}
)

add_library(${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    src/duration_recorder/duration_recorder.cpp
    src/rolling_recorder/rolling_recorder.cpp
    src/utils/file_utils.cpp
    src/utils/periodic_file_deleter.cpp
)

target_link_libraries(${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    ${catkin_LIBRARIES}
)

add_dependencies(${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(
    duration_recorder src/duration_recorder/main.cpp
)

target_link_libraries(duration_recorder
    ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
)

add_executable(
    rolling_recorder src/rolling_recorder/main.cpp
)

target_link_libraries(rolling_recorder
    ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    ${aws_common_INCLUDE_DIR}
    ${aws_ros1_common_INCLUDE_DIR}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS duration_recorder rolling_recorder ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
    set(TEST_INCLUDE_DIRS
      ${catkin_INCLUDE_DIRS}
      ${rosbag_cloud_recorders_INCLUDE_DIRS}
    )
    set(TEST_LINK_LIBS
      ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
      ${catkin_LIBRARIES}
    )
  find_package(rostest REQUIRED)
  find_package(GMock QUIET)

  if(GMOCK_FOUND)
    add_rostest_gmock(
      test_rolling_recorder
      test/test_rolling_recorder.test
      test/rolling_recorder_test.cpp
    )
    target_include_directories(test_rolling_recorder PRIVATE
      ${GMOCK_INCLUDE_DIRS}
    )
    target_link_libraries(test_rolling_recorder
      ${GMOCK_BOTH_LIBRARIES}
    )
  else()
    include_directories(/usr/include/gmock /usr/src/gmock)
    add_library(test_rolling_recorder_libgmock SHARED /usr/src/gmock/src/gmock-all.cc)
    add_rostest_gtest(test_rolling_recorder
      test/test_rolling_recorder.test
      test/rolling_recorder_test.cpp
    )
    target_link_libraries(test_rolling_recorder
      test_rolling_recorder_libgmock
    )
  endif()

  target_include_directories(test_rolling_recorder PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_rolling_recorder
    ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
  )

  find_common_test_packages()

  add_common_gtest(test_rolling_recorder_action_server_handler
    test/rolling_recorder_action_server_handler_test.cpp
  )

  add_common_gtest(test_duration_recorder_action_handler
    test/duration_recorder_action_server_handler_test.cpp
  )
  target_include_directories(test_duration_recorder_action_handler PRIVATE
      ${TEST_INCLUDE_DIRS}
  )
  target_link_libraries(test_duration_recorder_action_handler
      ${TEST_LINK_LIBS}
  )

  add_common_gtest(test_utils
    test/utils/utils_test.cpp
    test/utils/file_utils_test.cpp
    test/utils/periodic_file_deleter_test.cpp
    test/utils/rosbag_recorder.cpp
  )
  target_include_directories(test_utils
    PRIVATE include
  )
  target_include_directories(test_rolling_recorder_action_server_handler
    PRIVATE include
  )
  target_link_libraries(test_rolling_recorder_action_server_handler
    ${GTEST_LIBRARIES}
    ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
  )
  target_link_libraries(test_utils
    ${GTEST_LIBRARIES}
    ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
  )

endif()
