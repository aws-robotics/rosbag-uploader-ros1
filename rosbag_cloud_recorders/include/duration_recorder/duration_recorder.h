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
#include <recorder_common_error_codes.h>

namespace Aws
{
namespace Rosbag
{

typedef actionlib::ActionServer<recorder_msgs::DurationRecorderAction> DurationRecorderActionServer;
typedef actionlib::ActionServer<recorder_msgs::DurationRecorderAction>::GoalHandle GoalHandle;

/**
 *  Duration recorder is a node that responds to actions to record rosbag files
 */
class DurationRecorder
{
private:
  ros::NodeHandle node_handle_;
  DurationRecorderActionServer action_server_;
  std::unique_ptr<rosbag::Recorder> rosbag_recorder_;
  GoalHandle * current_goal_handle_;

  void ReleaseCurrentGoalHandle();
  void SetCurrentGoalHandle(DurationRecorderActionServer::GoalHandle & new_goal_handle);
  void GoalCallBack(DurationRecorderActionServer::GoalHandle goal);
  void CancelGoalCallBack(DurationRecorderActionServer::GoalHandle goal);
  void GenerateResult(recorder_msgs::DurationRecorderResult & recording_result, recorder_msgs::RecorderResult & t_recording_result, uint stage, std::string message);
  void GenerateFeedback(recorder_msgs::DurationRecorderFeedback & record_rosbag_action_feedback, uint stage);
  void ConfigureRecorderOptions(boost::shared_ptr<const recorder_msgs::DurationRecorderGoal> goal, rosbag::RecorderOptions recorder_options);
  int StartDurationRecorder(boost::shared_ptr<const recorder_msgs::DurationRecorderGoal> goal);

public:
  DurationRecorder();
  ~DurationRecorder() = default;
};

}  // namespace Rosbag
}  // namespace Aws
