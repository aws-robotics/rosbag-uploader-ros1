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

#include <rosbag_cloud_recorders/duration_recorder/duration_recorder_action_server_handler.h>

namespace Aws
{
namespace Rosbag
{

DurationRecorder::DurationRecorder() :
  node_handle_("~"),
  action_server_(node_handle_, "RosbagDurationRecord", false),
  rosbag_recorder_()
{
  action_server_.registerGoalCallback(
    [this](DurationRecorderActionServer::GoalHandle goal_handle) {
      DurationRecorderActionServerHandler<DurationRecorderActionServer::GoalHandle>::DurationRecorderStart(*rosbag_recorder_, goal_handle);
    }
  );

  action_server_.registerCancelCallback(
    [](DurationRecorderActionServer::GoalHandle goal_handle) {
      DurationRecorderActionServerHandler<DurationRecorderActionServer::GoalHandle>::CancelDurationRecorder(goal_handle);
    }
  );

  action_server_.start();
}

}  // namespace Rosbag
}  // namespace Aws
