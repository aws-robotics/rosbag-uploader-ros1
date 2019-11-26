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
    recorder_msgs
    roscpp
    rosbag
)

set(ROSBAG_CLOUD_RECORDER_LIB_TARGET ${PROJECT_NAME}_lib)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    CATKIN_DEPENDS actionlib actionlib_msgs recorder_msgs roscpp rosbag
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${AWSSDK_INCLUDE_DIR}
)

add_library(${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    src/duration_recorder/duration_recorder.cpp
    src/rolling_recorder/rolling_recorder.cpp
    src/utils/rosbag_file_manager.cpp
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
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS duration_recorder ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
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

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_duration_recorder
    test/test_duration_recorder.test
    test/duration_recorder_test.cpp
  )
  target_include_directories(test_duration_recorder PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${GTEST_LIBRARIES}
  )
  target_link_libraries(test_duration_recorder
    ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(test_rolling_recorder
    test/test_rolling_recorder.test
    test/rolling_recorder_test.cpp
  )
  target_include_directories(test_rolling_recorder PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_rolling_recorder
    ${ROSBAG_CLOUD_RECORDER_LIB_TARGET}
    ${catkin_LIBRARIES}
  )
endif()