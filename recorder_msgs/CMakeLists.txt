cmake_minimum_required(VERSION 2.8.3)
project(recorder_msgs)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
    std_msgs
)

## Generate actions in the action folder
add_action_files(
    DIRECTORY action
    FILES
    DurationRecorder.action
    RollingRecorder.action
)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    RecorderStatus.msg
    RecorderResult.msg
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package()
