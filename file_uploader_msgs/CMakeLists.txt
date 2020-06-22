cmake_minimum_required(VERSION 2.8.3)
project(file_uploader_msgs)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
    message_runtime
)

## Generate actions in the action folder
add_action_files(
    DIRECTORY action
    FILES
    UploadFiles.action
)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    S3Result.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package()
