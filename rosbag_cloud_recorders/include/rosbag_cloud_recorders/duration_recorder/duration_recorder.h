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

#pragma once

#include <actionlib/server/action_server.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <rosbag/recorder.h>

#include <recorder_msgs/DurationRecorderAction.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>

namespace Aws {
namespace Rosbag {

using DurationRecorderActionServer = actionlib::ActionServer<recorder_msgs::DurationRecorderAction>;

/**
 *  Duration recorder is a node that responds to actions to record rosbag files
 */
class DurationRecorder
{
public:
  DurationRecorder();
  ~DurationRecorder() = default;

private:
  void GoalCallBack(DurationRecorderActionServer::GoalHandle goal_handle);
  void CancelGoalCallBack(DurationRecorderActionServer::GoalHandle goal_handle);

  ros::NodeHandle node_handle_;
  DurationRecorderActionServer action_server_;
  std::unique_ptr<rosbag::Recorder> rosbag_recorder_;
};

}  // namespace Rosbag
}  // namespace Aws
