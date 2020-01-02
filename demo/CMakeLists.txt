cmake_minimum_required(VERSION 2.8.3)
project(demo)

set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    recorder_msgs
    roscpp
    std_msgs
    rosbag_cloud_recorders
    roscpp_serialization
)

set(DEMO_LIB_TARGET ${PROJECT_NAME}_lib)

catkin_package(
    LIBRARIES ${DEMO_LIB_TARGET}
    CATKIN_DEPENDS actionlib actionlib_msgs recorder_msgs roscpp rosbag_cloud_recorders roscpp_serialization
)

###########
## Build ##
###########

add_executable(
    demo src/main.cpp
)
target_link_libraries(demo ${catkin_LIBRARIES})
include_directories(${catkin_INCLUDE_DIRS})
#add_dependencies(demo actionlib actionlib_msgs recorder_msgs roscpp)

##target_link_libraries(demo
  ##  ${DEMO_LIB_TARGET}
##)

install(TARGETS demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_package()
