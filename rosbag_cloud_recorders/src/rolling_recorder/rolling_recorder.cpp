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

#include <fstream>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <rosbag/recorder.h>

#include <recorder_msgs/RollingRecorderAction.h>
#include <recorder_common_error_codes.h>
#include <rolling_recorder/rolling_recorder.h>


namespace Aws {
namespace Rosbag {

RollingRecorder::RollingRecorder() :
  node_handle_("~"),
  action_server_(node_handle_, "RosbagRollingRecord", false)
{
  action_server_.registerGoalCallback([this](RollingRecorderActionServer::GoalHandle goal_handle) {
    this->GoalCallBack(goal_handle);
  });
  action_server_.registerCancelCallback([this](RollingRecorderActionServer::GoalHandle goal_handle) {
    this->CancelGoalCallBack(goal_handle);
  });
}

void RollingRecorder::GoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle)
{
  goal_handle.setRejected();
}

void RollingRecorder::CancelGoalCallBack(RollingRecorderActionServer::GoalHandle /*goal_handle*/)
{
}

RecorderErrorCode RollingRecorder::StartRollingRecorder()
{
  bool successful_start = true;
  if (successful_start) {
    action_server_.start();
    return SUCCESS;
  } else {
    return FAILED;
  }
}

RecorderErrorCode RollingRecorder::StopRollingRecorder()
{
  return SUCCESS;
}

bool RollingRecorder::IsRollingRecorderActive()
{
  return false;
}

}  // namespace Rosbag
}  // namespace Aws
