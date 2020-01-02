/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <rolling_recorder/rolling_recorder.h>
#include <std_msgs/Duration.h>
#include <recorder_msgs/RollingRecorderAction.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<recorder_msgs::RollingRecorderAction> RollingRecorderActionClient;

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "test_rolling_recorder");
  RollingRecorderActionClient action_client("/rolling_recorder/RosbagRollingRecord", true);

  // ros::NodeHandle nh("~");
  // ros::AsyncSpinner executor(1);
  // executor.start();
  // action_client = std::make_shared<DurationRecorderActionClient>(nh, "RosbagDurationRecord/RosbagDurationRecord");
  //
  std::string destination("/opt/abbyxu/orgs/aws-robotics/rosbag-uploader-ros1");
  ros::Time start_time = ros::Time::now();
  ros::Time end_time = start_time + ros::Duration(29);

  recorder_msgs::RollingRecorderGoal goal;
  goal.destination = destination;
  goal.start_time = start_time;
  goal.end_time = end_time;

  ROS_INFO("sending goal");

  action_client.sendGoal(goal);
  // ROS_INFO(+goal_handle.getCommState().state_));
  ROS_INFO("done sending outout");
  // executor.stop();
  return 0;
}
