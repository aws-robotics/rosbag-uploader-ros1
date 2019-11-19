cmake_minimum_required(VERSION 2.8.3)
project(s3_file_uploader)

set(CMAKE_CXX_STANDARD 14)

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
# removed -Werror=pedantic until ros.h is fixed
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
    file_uploader_msgs
    roscpp
    s3_common
)

if(AWSSDK_FOUND)
  set(SERVICE s3)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE AWS_LINK_LIBS)
  link_directories("${AWSSDK_LIB_DIR}")
endif()

set(S3_UPLOADER_LIB_TARGET ${PROJECT_NAME}_lib)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${S3_UPLOADER_LIB_TARGET}
    CATKIN_DEPENDS actionlib actionlib_msgs file_uploader_msgs roscpp
)

###########
## Build ##
###########

include_directories(
    include
    ${AWSSDK_INCLUDE_DIR}
    ${actionlib_INCLUDE_DIRS}
    ${actionlib_msgs_INCLUDE_DIRS}
    ${aws_common_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${s3_common_INCLUDE_DIRS}
)

add_library(${S3_UPLOADER_LIB_TARGET}
    src/s3_file_uploader.cpp
)

target_link_libraries(${S3_UPLOADER_LIB_TARGET}
    ${AWS_LINK_LIBS}
    ${aws_common_LIBRARIES}
    ${catkin_LIBRARIES}
    ${s3_common_LIBRARIES}
)

add_dependencies(${S3_UPLOADER_LIB_TARGET}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${S3_UPLOADER_LIB_TARGET}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${S3_UPLOADER_LIB_TARGET}
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
    add_rostest_gtest(test_s3_file_uploader
        test/test_s3_file_uploader.test
        test/s3_file_uploader_test.cpp
    )
    target_include_directories(test_s3_file_uploader PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(test_s3_file_uploader
        ${S3_UPLOADER_LIB_TARGET}
        ${catkin_LIBRARIES}
    )
endif()

