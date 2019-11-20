cmake_minimum_required(VERSION 2.8.3)
project(rosbag_recorders)

set(CMAKE_CXX_STANDARD 14)

# Enable strict compiler flags if possible.
# include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
# removed -Werror=pedantic until ros.h is fixed
# set(FLAGS -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
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

set(ROSBAG_RECORDER_LIB_TARGET ${PROJECT_NAME}_lib)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${ROSBAG_RECORDER_LIB_TARGET}
    CATKIN_DEPENDS actionlib actionlib_msgs recorder_msgs roscpp rosbag
)

###########
## Build ##
###########

include_directories(
    include
    ${actionlib_INCLUDE_DIRS}
    ${actionlib_msgs_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${ROSBAG_RECORDER_LIB_TARGET}
    src/duration_recorder/duration_recorder.cpp
)

target_link_libraries(${ROSBAG_RECORDER_LIB_TARGET}
    ${catkin_LIBRARIES}
)

add_dependencies(${ROSBAG_RECORDER_LIB_TARGET}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}
    src/duration_recorder/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${ROSBAG_RECORDER_LIB_TARGET}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${ROSBAG_RECORDER_LIB_TARGET}
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
#TODO
