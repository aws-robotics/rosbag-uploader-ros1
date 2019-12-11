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
#include <string>
#include <vector>

namespace Aws
{
namespace Rosbag
{

RollingRecorder::RollingRecorder(
  ros::Duration bag_rollover_time, ros::Duration max_record_time, std::vector<std::string> topics_to_record) :
  node_handle_("~"),
  action_server_(node_handle_, "RosbagRollingRecord", false)
{
  rosbag::RecorderOptions rolling_recorder_options;
  rolling_recorder_options.max_duration = max_record_time;
  rolling_recorder_options.topics = topics_to_record;
  bag_rollover_time_ = bag_rollover_time;
  rosbag_rolling_recorder_ = std::make_unique<rosbag::Recorder>(rolling_recorder_options);
  action_server_.registerGoalCallback(
      boost::bind(&RollingRecorder::GoalCallBack, this, _1));
  action_server_.registerCancelCallback(
      boost::bind(&RollingRecorder::CancelGoalCallBack, this, _1));
}

void RollingRecorder::GoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle)
{
  goal_handle.setRejected();
}

void RollingRecorder::CancelGoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle)
{
  (void) goal_handle;
}

RecorderErrorCode RollingRecorder::StartRollingRecorder()
{
  bool successful_start = true;
  if (successful_start)
  {
    action_server_.start();
    return SUCCESS;
  }
  return FAILED;
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
