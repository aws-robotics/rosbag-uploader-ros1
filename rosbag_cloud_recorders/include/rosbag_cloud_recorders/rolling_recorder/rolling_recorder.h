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

#include <string>
#include <vector>

#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <recorder_msgs/RollingRecorderAction.h>
#include <file_uploader_msgs/UploadFilesAction.h>
#include <file_uploader_msgs/UploadFilesGoal.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <actionlib/client/simple_action_client.h>
#include <rosbag_cloud_recorders/utils/periodic_file_deleter.h>

namespace Aws
{
namespace Rosbag
{

using RollingRecorderActionServer = actionlib::ActionServer<recorder_msgs::RollingRecorderAction>;
using GoalHandle = actionlib::ActionServer<file_uploader_msgs::UploadFilesAction>::GoalHandle;
using UploadFilesActionSimpleClient = actionlib::SimpleActionClient<file_uploader_msgs::UploadFilesAction>;

struct RollingRecorderStatus {
  file_uploader_msgs::UploadFilesGoal current_upload_goal;
};

/**
 * Rolling recorder is a node that responds to actions to record rosbag files
 */
class RollingRecorder
{
public:
  explicit RollingRecorder(ros::Duration bag_rollover_time, ros::Duration max_record_time, std::string write_directory);

  virtual ~RollingRecorder() = default;

  /**
   * Returns a list of rosbag files that are no longer needed and can be deleted.
   * Used by the periodic_file_deleter.
   */
  std::vector<std::string> GetRosBagsToDelete() const;

  /**
   * Used by the callback handler to communicate information back to the recorder.
   */
  void UpdateStatus(RollingRecorderStatus status = RollingRecorderStatus());

private:
  void StartOldRosBagsPeriodicRemoval();
  file_uploader_msgs::UploadFilesGoal ConstructRosBagUploadGoal() const;
  RecorderErrorCode SendRosBagUploadGoal(const file_uploader_msgs::UploadFilesGoal & goal);

  ros::NodeHandle node_handle_;
  RollingRecorderActionServer action_server_;
  std::unique_ptr<UploadFilesActionSimpleClient> rosbag_uploader_action_client_;
  ros::Duration max_duration_;
  ros::Duration bag_rollover_time_;
  std::string write_directory_;
  Utils::PeriodicFileDeleter periodic_file_deleter_;
  RollingRecorderStatus status_;
};

}  // namespace Rosbag
}  // namespace Aws
