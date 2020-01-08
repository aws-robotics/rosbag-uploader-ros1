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

#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <rosbag/recorder.h>

#include <recorder_msgs/RollingRecorderAction.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws/core/utils/logging/LogMacros.h>

namespace Aws
{
namespace Rosbag
{

RollingRecorder::RollingRecorder(
  const ros::Duration & bag_rollover_time, const ros::Duration & max_record_time,
  std::vector<std::string> topics_to_record, std::string write_directory) :
  node_handle_("~"),
  action_server_(node_handle_, "RosbagRollingRecord", false),
  rosbag_uploader_action_client_(std::make_unique<UploadFilesActionSimpleClient>("/s3_file_uploader/UploadFiles", true)),
  max_duration_(max_record_time),
  is_rolling_recorder_running_(false),
  write_directory_(std::move(write_directory))
{
  rosbag::RecorderOptions rolling_recorder_options;
  rolling_recorder_options.max_duration = bag_rollover_time;
  rolling_recorder_options.topics = std::move(topics_to_record);
  rolling_recorder_options.split = true;
  rolling_recorder_options.prefix = write_directory_;
  rosbag_rolling_recorder_ = std::make_unique<rosbag::Recorder>(rolling_recorder_options);
  action_server_.registerGoalCallback([this](RollingRecorderActionServer::GoalHandle goal_handle) {
    this->GoalCallBack(goal_handle);
  });
  action_server_.registerCancelCallback([this](RollingRecorderActionServer::GoalHandle goal_handle) {
    this->CancelGoalCallBack(goal_handle);
  });
  action_server_.start();
}

RollingRecorder::RollingRecorder(
  const ros::Duration & bag_rollover_time, const ros::Duration & max_record_time, std::vector<std::string> topics_to_record)
  : RollingRecorder(bag_rollover_time, max_record_time, std::move(topics_to_record), "~/.ros/rosbag_uploader/")
{
}

void RollingRecorder::GoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle)
{
  goal_handle.setRejected();
}

void RollingRecorder::CancelGoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle)
{
  goal_handle.setCanceled();
}

RecorderErrorCode RollingRecorder::StartRollingRecorder()
{
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (IsRollingRecorderActive()) {
      AWS_LOG_WARN(__func__, "Rolling recorder already running.");
      return RECORDER_IS_RUNNING;
    }
    is_rolling_recorder_running_ = true;
  }

  std::string current_time = std::to_string(ros::Time::now().toSec());
  int recording_exit_code = rosbag_rolling_recorder_->run();

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  is_rolling_recorder_running_ = false;

  if (recording_exit_code != 0) {
    AWS_LOG_ERROR(__func__, "Ros bag starting at %s did not finish cleanly.", current_time.c_str());
    return FAILED;
  }
  return SUCCESS;
}

RecorderErrorCode RollingRecorder::StopRollingRecorder()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!IsRollingRecorderActive()) {
    return RECORDER_NOT_RUNNING;
  }
  is_rolling_recorder_running_ = false;
  return SUCCESS;
}

bool RollingRecorder::IsRollingRecorderActive() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return is_rolling_recorder_running_;
}

}  // namespace Rosbag
}  // namespace Aws
