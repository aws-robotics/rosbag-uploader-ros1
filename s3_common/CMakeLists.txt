cmake_minimum_required(VERSION 2.8.3)
project(s3_common)
set(S3_COMMON_VERSION 0.0.0)

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
)
target_link_libraries(${PROJECT_NAME} ${AWS_LINK_LIBS})
target_include_directories(${PROJECT_NAME}
  PRIVATE ${AWSSDK_INCLUDE_DIRS}
  include include ${aws_common_INCLUDE_DIRS})

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
