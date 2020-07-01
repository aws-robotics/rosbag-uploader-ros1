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
    aws_ros1_common
    file_uploader_msgs
    roscpp
    s3_common
)

find_package(Boost REQUIRED)

if(AWSSDK_FOUND)
  set(SERVICE s3)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE AWS_LINK_LIBS)
  link_directories("${AWSSDK_LIB_DIR}")
endif()

set(S3_UPLOADER_LIB_TARGET ${PROJECT_NAME}_lib)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${S3_UPLOADER_LIB_TARGET}
    CATKIN_DEPENDS actionlib actionlib_msgs aws_ros1_common file_uploader_msgs roscpp
    DEPENDS Boost
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

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

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
    set(TEST_TARGET test_s3_file_uploader)
    set(TEST_FILES
      test/test_s3_file_uploader.test
      test/s3_file_uploader_test.cpp
    )
    set(TEST_INCLUDE_DIRS
      ${catkin_INCLUDE_DIRS}
      ${s3_file_uploader_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
    )
    set(TEST_LINK_LIBS
      ${S3_UPLOADER_LIB_TARGET}
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
    )

    find_package(rostest REQUIRED)
    find_package(GMock QUIET)

    if(GMOCK_FOUND)
      add_rostest_gmock(${TEST_TARGET}
          ${TEST_FILES}
      )
      set(TEST_INCLUDE_DIRS
        ${TEST_INCLUDE_DIRS}
        ${GMOCK_INCLUDE_DIRS}
      )
      set(TEST_LINK_LIBS
        ${TEST_LINK_LIBS}
        ${GMOCK_BOTH_LIBRARIES}
      )

    else()
        include_directories(/usr/include/gmock /usr/src/gmock)
        add_library(${PROJECT_NAME}_libgmock SHARED /usr/src/gmock/src/gmock-all.cc)
        add_rostest_gtest(${TEST_TARGET}
          ${TEST_FILES}
        )
        set(TEST_LINK_LIBS
          ${TEST_LINK_LIBS}
          ${PROJECT_NAME}_libgmock
        )
    endif()

    find_common_test_packages()
    add_common_gtest(test_action_server_handler
      test/s3_file_uploader_action_server_handler_test.cpp
    )
    
    target_include_directories(test_action_server_handler PRIVATE
      ${TEST_INCLUDE_DIRS}
    )
    
    target_link_libraries(test_action_server_handler
      ${TEST_LINK_LIBS}
    )
    
    target_include_directories(${TEST_TARGET} PRIVATE
      ${TEST_INCLUDE_DIRS}
    )
    target_link_libraries(${TEST_TARGET}
      ${TEST_LINK_LIBS}
    )
    
endif()

