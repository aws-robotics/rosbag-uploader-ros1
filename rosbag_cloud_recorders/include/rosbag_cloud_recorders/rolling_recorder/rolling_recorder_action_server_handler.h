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
#include <memory>
#include <recorder_msgs/RollingRecorderAction.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder_action_server_handler.h>
#include <boost/filesystem.hpp>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <file_uploader_msgs/UploadFilesGoal.h>
#include <file_uploader_msgs/UploadFilesResult.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <actionlib/client/simple_action_client.h>
#include <rosbag_cloud_recorders/utils/file_utils.h>
#include <rosbag_cloud_recorders/utils/s3_client_utils.h>

namespace Aws {
namespace Rosbag {

template<typename GoalHandleT, typename UploadClientT>
struct RollingRecorderRosbagUploadRequest {
  GoalHandleT & goal_handle;
  const RollingRecorderOptions & rolling_recorder_options;
  UploadClientT & rosbag_uploader_action_client;
  std::atomic<bool> & action_server_busy;
  RollingRecorderStatus * recorder_status;
};

template<typename GoalHandleT, typename UploadClientT>
class RollingRecorderActionServerHandler
{
public:
  static void RollingRecorderRosbagUpload(
    const RollingRecorderRosbagUploadRequest<GoalHandleT, UploadClientT> & req
  ) {
    AWS_LOG_INFO(__func__, "A new goal has been recieved by the goal action server");
    bool expected_action_server_state = false;

    //  Check if action server is currently processing another goal
    if (std::atomic_compare_exchange_strong(&(req.action_server_busy),
                                            &expected_action_server_state,
                                            true)) {
      ProcessRollingRecorderGoal(req);
      req.action_server_busy = false;  // Done processing goal, setting action server status to not busy
    } else {
      const std::string log_message = "Rejecting new goal. Rolling recorder is already processing a goal.";
      recorder_msgs::RollingRecorderResult result;
      Utils::GenerateResult(recorder_msgs::RecorderResult::INVALID_INPUT, log_message, result);
      req.goal_handle.setRejected(result, log_message);
      AWS_LOG_WARN(__func__, log_message.c_str());
    }
  }

private:
  static void ProcessRollingRecorderGoal(
    const RollingRecorderRosbagUploadRequest<GoalHandleT, UploadClientT> & req
  ) {
    recorder_msgs::RollingRecorderResult result;
    ros::Time time_of_goal_received = ros::Time::now();

    //  Accept incoming goal and start processing it
    AWS_LOG_INFO(__func__, "Sending rosbag uploader goal to uploader action server.");
    req.goal_handle.setAccepted();

    recorder_msgs::RollingRecorderFeedback recorder_feedback;
    recorder_msgs::RecorderStatus recording_status;
    Utils::GenerateFeedback(
      recorder_msgs::RecorderStatus::PREPARING_UPLOAD,
      time_of_goal_received,
      recorder_feedback,
      recording_status);
    req.goal_handle.publishFeedback(recorder_feedback);
    std::vector<std::string> rosbags_to_upload = Utils::GetRosbagsToUpload(req.rolling_recorder_options.write_directory,
      [time_of_goal_received](rosbag::View& rosbag) -> bool
      {
        return time_of_goal_received >= rosbag.getBeginTime();
      }
    );
    if (rosbags_to_upload.empty()) {
      const std::string msg = "No rosbags found to upload.";
      Utils::GenerateResult(recorder_msgs::RecorderResult::SUCCESS, msg, result);
      req.goal_handle.setSucceeded(result, msg);
      AWS_LOG_INFO(__func__, msg.c_str());
      return;
    }

    auto goal = Utils::ConstructRosBagUploaderGoal(req.goal_handle.getGoal()->destination, rosbags_to_upload);
    req.recorder_status->SetUploadGoal(goal);
    req.rosbag_uploader_action_client.sendGoal(goal);

    Utils::GenerateFeedback(
      recorder_msgs::RecorderStatus::UPLOADING,
      ros::Time::now(),
      recorder_feedback,
      recording_status);
    req.goal_handle.publishFeedback(recorder_feedback);
    bool upload_finished = req.rosbag_uploader_action_client.waitForResult(ros::Duration(req.rolling_recorder_options.upload_timeout_s));

    Utils::HandleRecorderUploadResult(req.goal_handle, req.rosbag_uploader_action_client.getState(), upload_finished, result);
    req.recorder_status->SetUploadGoal(file_uploader_msgs::UploadFilesGoal());
  }
};

}  // namespace Rosbag
}  // namespace Aws
