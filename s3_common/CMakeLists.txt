cmake_minimum_required(VERSION 2.8.3)
project(s3_common)
set(S3_COMMON_VERSION 0.1.0)

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

find_package(Boost REQUIRED)
find_package(aws_common REQUIRED)

if(AWSSDK_FOUND)
  set(SERVICE s3)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE AWS_LINK_LIBS)
  link_directories("${AWSSDK_LIB_DIR}")
endif()

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
    src/s3_facade.cpp
    src/s3_upload_manager.cpp
    src/utils.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${AWS_LINK_LIBS}
  ${aws_common_LIBRARIES}
  ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${AWSSDK_INCLUDE_DIRS}
  ${aws_common_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}-targets"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

export(TARGETS ${PROJECT_NAME}
        FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(${PROJECT_NAME}Config.cmake.in
               ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(${PROJECT_NAME}ConfigVersion.cmake.in
               ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(EXPORT ${PROJECT_NAME}-targets DESTINATION share/${PROJECT_NAME}/cmake)

##########
## Test ##
##########

enable_testing()
find_common_test_packages()

add_common_gtest(test_s3_common
  test/main_test.cpp
  test/s3_facade_test.cpp
  test/s3_upload_manager_test.cpp
  test/utils_test.cpp
)

if(TARGET test_s3_common)
  target_include_directories(test_s3_common
    PRIVATE include
    PRIVATE ${aws_common_INCLUDE_DIRS}
  )

  target_link_libraries(test_s3_common
    ${GMOCK_LIBRARY}
    ${GTEST_LIBRARIES}
    ${PROJECT_NAME}
  )

  add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND $(MAKE) tests
  )
endif()
