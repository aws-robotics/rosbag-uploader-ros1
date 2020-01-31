cmake_minimum_required(VERSION 2.8.3)
project(rosbag_cloud_recorders)

set(CMAKE_CXX_STANDARD 14)

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
# Removed -Werror=pedantic until ros.h is fixed
set(FLAGS -Wno-long-long -Wall -Wextra -Wcast-align -Wformat -Wwrite-strings)
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
    std_msgs
    roslint
)
catkin_package()

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
            include/duration_recorder/duration_recorder_action_server_handler.h)

add_library(${PROJECT_NAME}_lib
    src/duration_recorder/duration_recorder.cpp
    src/rolling_recorder/rolling_recorder.cpp
    src/utils/file_utils.cpp
    src/utils/periodic_file_deleter.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})

add_executable(duration_recorder src/duration_recorder/main.cpp)
#target_include_directories(duration_recorder PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(duration_recorder ${PROJECT_NAME}_lib)

add_executable(rolling_recorder src/rolling_recorder/main.cpp)
#target_include_directories(rolling_recorder PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(rolling_recorder ${PROJECT_NAME}_lib)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS duration_recorder rolling_recorder ${PROJECT_NAME}_lib
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
    include
    ${catkin_INCLUDE_DIRS}
    ${rosbag_cloud_recorders_INCLUDE_DIRS}
  )
  set(TEST_LINK_LIBS
    ${PROJECT_NAME}_lib
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

  target_include_directories(test_rolling_recorder PRIVATE ${TEST_INCLUDE_DIRS})
  target_link_libraries(test_rolling_recorder
    ${PROJECT_NAME}_lib
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
  )
  target_include_directories(test_utils PRIVATE ${TEST_INCLUDE_DIRS})
  target_include_directories(test_rolling_recorder_action_server_handler PRIVATE ${TEST_INCLUDE_DIRS})
  target_link_libraries(test_rolling_recorder_action_server_handler
    ${GTEST_LIBRARIES}
    ${PROJECT_NAME}_lib
  )
  target_link_libraries(test_utils
    ${GTEST_LIBRARIES}
    ${PROJECT_NAME}_lib
  )

endif()
