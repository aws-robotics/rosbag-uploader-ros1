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

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <ros/ros.h>

#include <recorder_msgs/RollingRecorderAction.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder_action_server_handler.h>

#include <rosbag_cloud_recorders/utils/file_utils.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws/core/utils/logging/LogMacros.h>

namespace Aws
{
namespace Rosbag
{
RollingRecorder::RollingRecorder() :
  node_handle_("~"),
  action_server_(node_handle_, "RosbagRollingRecord", false),
  rosbag_uploader_action_client_("/s3_file_uploader/UploadFiles", true),
  periodic_file_deleter_([this]()->std::vector<std::string>{return this->GetRosBagsToDelete();}, rolling_recorder_options_.bag_rollover_time.toSec()),
  action_server_busy_(false) {}

bool RollingRecorder::ValidInputParam(Aws::Rosbag::RollingRecorderOptions rolling_recorder_options) {
  return (rolling_recorder_options.bag_rollover_time.toSec() > 0) && (rolling_recorder_options.max_record_time.toSec() > 0) && (rolling_recorder_options.bag_rollover_time.toSec() <= rolling_recorder_options.max_record_time.toSec());
}

void RollingRecorder::InitializeRollingRecorder(RollingRecorderOptions rolling_recorder_options) {
  rolling_recorder_options_ = std::move(rolling_recorder_options);

  if (!ValidInputParam(rolling_recorder_options_)) {
    AWS_LOG_ERROR(__func__, "Failed to start rolling recorder due to bag_rollover_time and max_record_time are invalid.");
    ros::shutdown();
  }
  action_server_.registerGoalCallback([&](RollingRecorderActionServer::GoalHandle goal_handle) {
    RollingRecorderActionServerHandler<RollingRecorderActionServer::GoalHandle, UploadFilesActionSimpleClient>::RollingRecorderRosbagUpload(goal_handle,
      rolling_recorder_options_, rosbag_uploader_action_client_, action_server_busy_);
  });
  action_server_.start();
  periodic_file_deleter_.Start();
}

void RollingRecorder::UpdateStatus(RollingRecorderStatus status) {
  status_ = std::move(status);
}

std::vector<std::string> RollingRecorder::GetRosBagsToDelete() const
{
  AWS_LOG_DEBUG(__func__, "Getting ros bags to delete");
  boost::filesystem::path dir_path(rolling_recorder_options_.write_directory);
  std::vector<std::string> delete_files;
  boost::system::error_code ec;
  for (boost::filesystem::directory_iterator itr(dir_path, ec);
       itr != boost::filesystem::directory_iterator(); ++itr) {
    if (ec.value() != 0) {
      AWS_LOGSTREAM_INFO(__func__, "Getting rosbags to delete errored with message: " << ec.message());
      break;
    }
    if (itr->path().extension().string() != ".bag") {
      continue;
    }
    auto file_path = itr->path().string();
    if (status_.current_upload_goal.files.end() != std::find(status_.current_upload_goal.files.begin(),
                                                             status_.current_upload_goal.files.end(),
                                                             file_path)) {
      AWS_LOGSTREAM_DEBUG(__func__, "Skipping deletion of upload candidate: " << file_path);
      continue;
    }
    AWS_LOGSTREAM_DEBUG(__func__, "Checking path: " << file_path);
    auto bag_start_time = Utils::GetRosBagStartTime(file_path);
    AWS_LOGSTREAM_DEBUG(__func__, "Bag start time is: "<< bag_start_time);
    if (bag_start_time != ros::Time(0) && ros::Time::now() - bag_start_time > rolling_recorder_options_.max_record_time) {
      AWS_LOGSTREAM_DEBUG(__func__, "Marking file for deletion: " << file_path);
      delete_files.emplace_back(file_path);
    }
  }
  return delete_files;
}

}  // namespace Rosbag
}  // namespace Aws
