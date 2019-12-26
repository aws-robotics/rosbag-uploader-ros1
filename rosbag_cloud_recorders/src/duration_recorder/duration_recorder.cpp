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

#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <ros/ros.h>
#include <rosbag/recorder.h>

#include <recorder_msgs/DurationRecorderAction.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <rosbag_cloud_recorders/duration_recorder/duration_recorder.h>

namespace Aws
{
namespace Rosbag
{

DurationRecorder::DurationRecorder() :
    node_handle_("~"),
    action_server_(node_handle_, "RosbagDurationRecord", false)
{
  rosbag_recorder_ = nullptr;
  action_server_.registerGoalCallback(
      boost::bind(&DurationRecorder::GoalCallBack, this, _1));
  action_server_.registerCancelCallback(
      boost::bind(&DurationRecorder::CancelGoalCallBack, this, _1));
  action_server_.start();
}

void DurationRecorder::GoalCallBack(DurationRecorderActionServer::GoalHandle goal_handle)
{
  goal_handle.setRejected();
}

void DurationRecorder::CancelGoalCallBack(DurationRecorderActionServer::GoalHandle goal_handle)
{
  (void) goal_handle;
}

}  // namespace Rosbag
}  // namespace Aws
