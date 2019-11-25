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

#include <recorder_msgs/RollingRecorderAction.h>
#include <recorder_common_error_codes.h>


namespace Aws {
namespace Rosbag {

typedef actionlib::ActionServer<recorder_msgs::RollingRecorderAction> RollingRecorderActionServer;

/**
 * Rolling recorder is a node that responds to actions to record rosbag files
 */
class RollingRecorder
{
public:
  RollingRecorder();
  ~RollingRecorder() = default;

  /**
   * Activate the rolling recorder so that it is recording rosbags in the background
   */
  void Activate();

  /**
   * Deactivate the rolling recorder so that it stops recording rosbags in the background
   */
  void Deactivate();

  /**
   * Returns boolean indicating whether or not the rolling recorder is currently recording in the
   * background
   */
  bool IsActive();

private:
  void GoalCallBack(RollingRecorderActionServer::GoalHandle goal);
  void CancelGoalCallBack(RollingRecorderActionServer::GoalHandle goal);

  ros::NodeHandle node_handle_;
  RollingRecorderActionServer action_server_;
  std::unique_ptr<rosbag::Recorder> rosbag_recorder_;
};

}  // namespace Rosbag
}  // namespace Aws
