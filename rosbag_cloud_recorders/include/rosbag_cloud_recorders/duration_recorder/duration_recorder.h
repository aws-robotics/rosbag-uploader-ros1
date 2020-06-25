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
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <file_uploader_msgs/UploadFilesAction.h>
#include <recorder_msgs/DurationRecorderAction.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <rosbag_cloud_recorders/utils/recorder.h>
#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>

namespace Aws {
namespace Rosbag {

// Will contain option for deleting the rosbag after upload
struct DurationRecorderOptions
{
  uint64_t min_free_space_mib {0};  // minimum free disk space in mebibytes
  std::string write_directory;
  double upload_timeout_s {0};
  bool delete_bags_after_upload {false};
};

using DurationRecorderActionServer = actionlib::ActionServer<recorder_msgs::DurationRecorderAction>;
using S3FileUploaderSimpleActionClient = actionlib::SimpleActionClient<file_uploader_msgs::UploadFilesAction>;
/**
 *  Duration recorder is a node that responds to actions to record rosbag files
 */
class DurationRecorder
{
public:
  DurationRecorder();
  explicit DurationRecorder(DurationRecorderOptions duration_recorder_options);
  ~DurationRecorder() = default;

private:
  DurationRecorderOptions duration_recorder_options_;
  ros::NodeHandle node_handle_;
  DurationRecorderActionServer action_server_;
  S3FileUploaderSimpleActionClient upload_client_;
  std::unique_ptr<Utils::RosbagRecorder<Utils::Recorder>> rosbag_recorder_;

};

}  // namespace Rosbag
}  // namespace Aws
