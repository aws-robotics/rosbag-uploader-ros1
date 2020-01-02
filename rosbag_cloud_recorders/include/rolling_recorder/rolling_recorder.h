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

#ifndef ROLLING_RECORDER_ROLLING_RECORDER_H
#define ROLLING_RECORDER_ROLLING_RECORDER_H

#pragma once

#include <actionlib/server/action_server.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <rosbag/recorder.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <recorder_msgs/RollingRecorderAction.h>
#include <file_uploader_msgs/UploadFilesAction.h>
#include <file_uploader_msgs/UploadFilesGoal.h>
#include <recorder_common_error_codes.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include <unordered_map>
#include <string>
#include <vector>

namespace Aws
{
namespace Rosbag
{

using RollingRecorderActionServer = actionlib::ActionServer<recorder_msgs::RollingRecorderAction>;
using UploadFilesGoalHandle = actionlib::ActionServer<file_uploader_msgs::UploadFilesAction>::GoalHandle;
using UploadFilesActionSimpleClient = actionlib::SimpleActionClient<file_uploader_msgs::UploadFilesAction>;

/**
 * Rolling recorder is a node that responds to actions to record rosbag files
 */
class RollingRecorder
{
public:
  RollingRecorder(
    ros::Duration bag_rollover_time, ros::Duration max_record_time, std::vector<std::string> topics_to_record);

  RollingRecorder(
    ros::Duration bag_rollover_time, ros::Duration max_record_time,
    std::vector<std::string> topics_to_record, std::string write_directory);

  ~RollingRecorder() = default;

  /**
   * Activate the rolling recorder so that it is recording rosbags in the background
   */
  RecorderErrorCode StartRollingRecorder();

  /**
   * Deactivate the rolling recorder so that it stops recording rosbags in the background
   */
  RecorderErrorCode StopRollingRecorder();

  /**
   * Returns boolean indicating whether or not the rolling recorder is currently recording in the
   * background
   */
  bool IsRollingRecorderActive() const;

private:
  void GoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle);
  void CancelGoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle);
  std::vector<std::string> GetRosgBagListToDelete() const;
  std::vector<std::string> GetRosgBagListToUplaod() const;
  void StartOldRosBagsPeriodicRemoval();
  file_uploader_msgs::UploadFilesGoal ConstructRosBagUploaderGoal(std::string destination) const;
  RecorderErrorCode SendRosBagUploaderGoal(const file_uploader_msgs::UploadFilesGoal & goal);
  void ReleaseCurrentGoalHandle();
  void SetCurrentGoalHandle(RollingRecorderActionServer::GoalHandle & new_goal_handle);
  void GenerateResult(recorder_msgs::RollingRecorderResult & recording_result, recorder_msgs::RecorderResult & t_recording_result, uint stage, std::string message);
  void GenerateFeedback(recorder_msgs::RollingRecorderFeedback & record_rosbag_action_feedback, uint stage);
  bool ValidateGoal(boost::shared_ptr<const recorder_msgs::RollingRecorderGoal> goal, double current_time_in_sec);

  enum LocalRosBagStatus { EXPIRED, ACTIVE, UPLOADED };

  std::unordered_map<std::string, Aws::Rosbag::RollingRecorder::LocalRosBagStatus> local_rosbag_status_;
  ros::NodeHandle node_handle_;
  RollingRecorderActionServer action_server_;
  std::unique_ptr<rosbag::Recorder> rosbag_rolling_recorder_;
  std::unique_ptr<UploadFilesActionSimpleClient> rosbag_uploader_action_client_;
  ros::Duration max_duration_;
  RollingRecorderActionServer::GoalHandle * current_goal_handle_;
  bool is_rolling_recorder_running_;
  bool is_stop_rolling_recorder_called_;
  std::string write_directory_;
  rosbag::RecorderOptions rolling_recorder_options_;
};

}  // namespace Rosbag
}  // namespace Aws

#endif  // ROLLING_RECORDER_ROLLING_RECORDER_H
