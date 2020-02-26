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
  upload_request_data_(std::make_shared<UploadRequestData>("/s3_file_uploader/UploadFiles", true)) {}

bool RollingRecorder::ValidInputParam(Aws::Rosbag::RollingRecorderOptions rolling_recorder_options) {
  if (rolling_recorder_options.bag_rollover_time.toSec() <= 0) {
    AWS_LOG_ERROR(__func__, "bag_rollover_time must be a positive integer.");
    return false;
  }
  if (rolling_recorder_options.max_record_time.toSec() <= 0) {
    AWS_LOG_ERROR(__func__, "max_record_time must be a positive integer.");
    return false;
  }
  if (rolling_recorder_options.bag_rollover_time.toSec() > rolling_recorder_options.max_record_time.toSec()) {
    AWS_LOG_ERROR(__func__, "bag_rollover_time cannot be greater than max_record_time.");
    return false;
  }
  if (rolling_recorder_options.upload_timeout_s <= 0) {
    AWS_LOG_ERROR(__func__, "upload_timeout_s must be a positive number.");
    return false;
  }
  return true;
}

bool RollingRecorder::InitializeRollingRecorder(RollingRecorderOptions rolling_recorder_options) {
  upload_request_data_->rolling_recorder_options_ = std::move(rolling_recorder_options);
  if (!ValidInputParam(upload_request_data_->rolling_recorder_options_)) {
    return false;
  }

  periodic_file_deleter_ = std::make_unique<Utils::PeriodicFileDeleter>(
    [this]()->std::vector<std::string>{
      return this->GetRosBagsToDelete();
    },
    upload_request_data_->rolling_recorder_options_.bag_rollover_time.toSec());

  std::weak_ptr<UploadRequestData> data_weak_ptr = upload_request_data_;
  action_server_.registerGoalCallback([data_weak_ptr](RollingRecorderActionServer::GoalHandle goal_handle) {
    std::shared_ptr<UploadRequestData> data_ptr = data_weak_ptr.lock();
    if (nullptr == data_ptr) {
      return;
    }
    RollingRecorderRosbagUploadRequest<RollingRecorderActionServer::GoalHandle, UploadFilesActionSimpleClient> request{
      .goal_handle = goal_handle,
      .rolling_recorder_options = data_ptr->rolling_recorder_options_,
      .rosbag_uploader_action_client = data_ptr->rosbag_uploader_action_client_,
      .action_server_busy = data_ptr->action_server_busy_,
      .recorder_status = &data_ptr->recorder_status_
    };
    RollingRecorderActionServerHandler<RollingRecorderActionServer::GoalHandle, UploadFilesActionSimpleClient>::RollingRecorderRosbagUpload(request);
  });

  action_server_.start();
  periodic_file_deleter_->Start();

  return true;
}

void RollingRecorder::UpdateStatus(const RollingRecorderStatus & status)
{
  upload_request_data_->recorder_status_ = status;
}

std::vector<std::string> RollingRecorder::GetRosBagsToDelete() const
{
  AWS_LOG_DEBUG(__func__, "Getting ros bags to delete");
  boost::filesystem::path dir_path(upload_request_data_->rolling_recorder_options_.write_directory);
  std::vector<std::string> delete_files;
  boost::system::error_code ec;
  for (boost::filesystem::directory_iterator itr(dir_path, ec);
       itr != boost::filesystem::directory_iterator(); ++itr) {
    if (ec.value() != 0) {
      AWS_LOGSTREAM_WARN(__func__, "boost::filesystem::directory_iterator errored with message: " << ec.message());
      break;
    }
    if (itr->path().extension().string() != ".bag") {
      continue;
    }
    const auto & file_path = itr->path().string();
    const auto & file_paths = upload_request_data_->recorder_status_.GetUploadGoal().files;
    if (file_paths.end() != std::find(file_paths.begin(), file_paths.end(), file_path)) {
      AWS_LOGSTREAM_DEBUG(__func__, "Skipping deletion of upload candidate: " << file_path);
      continue;
    }
    AWS_LOGSTREAM_DEBUG(__func__, "Checking path: " << file_path);
    auto bag_start_time = Utils::GetRosBagStartTime(file_path);
    AWS_LOGSTREAM_DEBUG(__func__, "Bag start time is: "<< bag_start_time);
    if (bag_start_time != ros::Time(0) && ros::Time::now() - bag_start_time > 
      upload_request_data_->rolling_recorder_options_.max_record_time) {
      AWS_LOGSTREAM_DEBUG(__func__, "Marking file for deletion: " << file_path);
      delete_files.emplace_back(file_path);
    }
  }
  return delete_files;
}

}  // namespace Rosbag
}  // namespace Aws
