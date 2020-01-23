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

#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
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

RollingRecorder::RollingRecorder(ros::Duration bag_rollover_time,
  ros::Duration max_record_time, std::string write_directory) :
  node_handle_("~"),
  action_server_(node_handle_, "RosbagRollingRecord", false),
  rosbag_uploader_action_client_(std::make_unique<UploadFilesActionSimpleClient>("/s3_file_uploader/UploadFiles", true)),
  max_duration_(std::move(max_record_time)),
  bag_rollover_time_(std::move(bag_rollover_time)),
  write_directory_(std::move(write_directory)),
  periodic_file_deleter_([this]()->std::vector<std::string>{return this->GetRosBagsToDelete();}, bag_rollover_time.toSec())
{
  action_server_.registerGoalCallback([&](RollingRecorderActionServer::GoalHandle goal_handle) {
    RollingRecorderActionServerHandler<RollingRecorderActionServer::GoalHandle>::RollingRecorderRosbagUpload(goal_handle, write_directory_, bag_rollover_time_);
  });
  action_server_.registerCancelCallback([](RollingRecorderActionServer::GoalHandle goal_handle) {
    RollingRecorderActionServerHandler<RollingRecorderActionServer::GoalHandle>::CancelRollingRecorderRosbagUpload(goal_handle);
  });
  action_server_.start();
  periodic_file_deleter_.Start();
}

std::vector<std::string> RollingRecorder::GetRosBagsToDelete() const
{
  AWS_LOG_DEBUG(__func__, "Getting ros bags to delete");
  boost::filesystem::path path(write_directory_);
  std::vector<std::string> delete_files;
  for (boost::filesystem::directory_iterator itr(path);
       itr != boost::filesystem::directory_iterator(); ++itr) {
    auto path = itr->path().string();
    AWS_LOGSTREAM_DEBUG(__func__, "Checking path: " << path);
    auto bag_start_time = Utils::GetRosBagStartTime(path);
    AWS_LOGSTREAM_DEBUG(__func__, "Bag start time is: "<< bag_start_time);
    // Need to add check path isn't in current goal vector once that's implemented
    if (bag_start_time != ros::Time(0) && ros::Time::now() - bag_start_time > max_duration_) {
      AWS_LOGSTREAM_DEBUG(__func__, "Marking file for deletion: " << path);
      delete_files.emplace_back(path);
    }
  }
  return delete_files;
}

}  // namespace Rosbag
}  // namespace Aws
