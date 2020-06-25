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
#include <atomic>
#include <ros/ros.h>

#include <recorder_msgs/RollingRecorderAction.h>
#include <file_uploader_msgs/UploadFilesAction.h>
#include <file_uploader_msgs/UploadFilesGoal.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <rosbag_cloud_recorders/utils/periodic_file_deleter.h>

namespace Aws
{
namespace Rosbag
{

using RollingRecorderActionServer = actionlib::ActionServer<recorder_msgs::RollingRecorderAction>;
using UploadFilesActionSimpleClient = actionlib::SimpleActionClient<file_uploader_msgs::UploadFilesAction>;

// Contains option for deleting the rosbag after upload
struct RollingRecorderOptions
{
  uint64_t min_free_space_mib {0};  // minimum free disk space in mebibytes
  std::string write_directory;
  double upload_timeout_s {0};
  ros::Duration max_record_time;
  ros::Duration bag_rollover_time;
};

class RollingRecorderStatus {
public:
  virtual void SetUploadGoal(const file_uploader_msgs::UploadFilesGoal & goal)
  {
    current_upload_goal_ = goal;
  }

  const file_uploader_msgs::UploadFilesGoal & GetUploadGoal() const
  {
    return current_upload_goal_;
  }

private:
  file_uploader_msgs::UploadFilesGoal current_upload_goal_;
};

/**
 * Rolling recorder is a node that responds to actions to record rosbag files
 */
class RollingRecorder
{
public:
  explicit RollingRecorder();
  RollingRecorder(const RollingRecorder & other) = delete;
  RollingRecorder(RollingRecorder && other) = delete;
  RollingRecorder & operator=(const RollingRecorder & other) = delete;
  RollingRecorder & operator=(RollingRecorder && other) = delete;
  virtual ~RollingRecorder() = default;

  /**
   * Returns a list of rosbag files that are no longer needed and can be deleted.
   * Used by the periodic_file_deleter.
   */
  std::vector<std::string> GetRosBagsToDelete() const;

  /**
   * Used by the callback handler to communicate information back to the recorder.
   */
  void UpdateStatus(const RollingRecorderStatus & status);

  /**
   * To determine whether to start the rolling recorder action server based on whether the rolling recorder options are valid
   */
  bool ValidInputParam(RollingRecorderOptions rolling_recorder_options);

  /**
   * Initialize the rolling recorder action server
   */
  bool InitializeRollingRecorder(RollingRecorderOptions rolling_recorder_options);

private:
  struct UploadRequestData {
    UploadRequestData(const std::string & client_name, bool spin_thread)
      : rosbag_uploader_action_client_(client_name, spin_thread), action_server_busy_(false) {}

    RollingRecorderOptions rolling_recorder_options_;
    UploadFilesActionSimpleClient rosbag_uploader_action_client_;
    std::atomic<bool> action_server_busy_;
    RollingRecorderStatus recorder_status_;
  };

  void StartOldRosBagsPeriodicRemoval();
  void InitializeRollingRecorder();

  ros::NodeHandle node_handle_;
  RollingRecorderActionServer action_server_;
  std::shared_ptr<UploadRequestData> upload_request_data_;
  std::unique_ptr<Utils::PeriodicFileDeleter> periodic_file_deleter_;
};

}  // namespace Rosbag
}  // namespace Aws
