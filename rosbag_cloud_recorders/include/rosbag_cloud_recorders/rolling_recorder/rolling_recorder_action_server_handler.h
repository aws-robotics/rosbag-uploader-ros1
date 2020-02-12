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
#include <rosbag_cloud_recorders/utils/file_utils.h>
#include <boost/filesystem.hpp>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <file_uploader_msgs/UploadFilesGoal.h>
#include <file_uploader_msgs/UploadFilesResult.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <actionlib/client/simple_action_client.h>

namespace Aws{
namespace Rosbag{

template<typename GoalHandleT, typename SimpleActionClientT>
class RollingRecorderActionServerHandler
{
public:
  static void RollingRecorderRosbagUpload(GoalHandleT& goal_handle,
    const RollingRecorderOptions& rolling_recorder_options, std::shared_ptr<SimpleActionClientT>& rosbag_uploader_action_client,
    std::atomic<bool>& action_server_busy)
  {
    AWS_LOG_INFO(__func__, "A new goal has been recieved by the goal action server");
    std::string log_message;
    bool expected_action_server_state = false;

    //  Check if action server is currently processing another goal
    if (std::atomic_compare_exchange_strong(&action_server_busy, &expected_action_server_state, true)) {
      ProcessRollingRecorderGoal(goal_handle, rolling_recorder_options, rosbag_uploader_action_client, log_message);
      action_server_busy = false;  // Done processing goal, setting action server status to not busy
    } else {
      log_message = "Rejecting new goal. Rolling recorder is already processing a goal.";
      AWS_LOG_WARN(__func__, log_message.c_str());
      goal_handle.setRejected(GenerateResult(recorder_msgs::RecorderResult::INVALID_INPUT, log_message), log_message);
    }
  }

private:
  static void ProcessRollingRecorderGoal(GoalHandleT& goal_handle,
    const RollingRecorderOptions& rolling_recorder_options, std::shared_ptr<SimpleActionClientT>& rosbag_uploader_action_client, std::string & log_message)
  {
    ros::Time time_of_goal_received = ros::Time::now();
    //  Attempt to connect S3 uploader action server
    rosbag_uploader_action_client->waitForServer(ros::Duration(rolling_recorder_options.upload_timeout_s));
    if (!rosbag_uploader_action_client->isServerConnected()) {
      log_message = "Not able to connect to file uploader action server, rosbags uploading failed to complete.";
      AWS_LOG_WARN(__func__, log_message.c_str());
      goal_handle.setAborted(GenerateResult(recorder_msgs::RecorderResult::INTERNAL_ERROR, log_message), log_message);
      return;
    }

    //  Accept incoming goal and start processing it
    goal_handle.setAccepted();
    recorder_msgs::RollingRecorderFeedback record_rosbag_action_feedback;
    GenerateFeedback(recorder_msgs::RecorderStatus::UPLOADING);
    goal_handle.publishFeedback(record_rosbag_action_feedback);
    AWS_LOG_INFO(__func__, "Sending rosbag uploader goal to uploader action server.");

    int return_code;
    //  Get list of rosbags to upload
    std::vector<std::string> ros_bags_to_upload = Utils::GetRosbagsToUpload(rolling_recorder_options.write_directory,
      [time_of_goal_received](rosbag::View& rosbag) -> bool
      {
        return time_of_goal_received >= rosbag.getBeginTime();
      }
    );

    //  Send acquired list of rosbags to s3 file uploader and have uploader uploads these files
    file_uploader_msgs::UploadFilesGoal file_uploader_goal = ConstructRosBagUploaderGoal(goal_handle.getGoal()->destination, ros_bags_to_upload);
    RecorderErrorCode upload_status;
    upload_status = SendRosBagUploaderGoal(file_uploader_goal, rosbag_uploader_action_client, return_code, rolling_recorder_options.upload_timeout_s);

    //  Check if operation is successful
    if (UPLOADING_TIMED_OUT == upload_status) {
      log_message = "Rosbags uploading to S3 timed out.";
      AWS_LOG_WARN(__func__, log_message.c_str());
      goal_handle.setAborted(GenerateResult(upload_status, log_message), log_message);
      return;
    }

    if (return_code != 200) {
      log_message = "Uploading rosbags failed to finish, S3 error message: " + rosbag_uploader_action_client->getState().getText();

      AWS_LOG_WARN(__func__, log_message.c_str());
      goal_handle.setAborted(GenerateResult(upload_status, log_message), log_message);
      return;
    }

    log_message = "A batch of rosbag  were uploaded successfully.";
    AWS_LOG_INFO(__func__, log_message.c_str());
    goal_handle.setSucceeded(GenerateResult(recorder_msgs::RecorderResult::SUCCESS, log_message), log_message);
  }

  static file_uploader_msgs::UploadFilesGoal ConstructRosBagUploaderGoal(std::string destination, std::vector<std::string> & ros_bags_to_upload)
  {
    AWS_LOG_INFO(__func__, "Constructing Uploader Goal.");
    file_uploader_msgs::UploadFilesGoal file_uploader_goal;
    file_uploader_goal.files = ros_bags_to_upload;
    file_uploader_goal.upload_location = std::move(destination);
    return file_uploader_goal;
  }

  static RecorderErrorCode SendRosBagUploaderGoal(const file_uploader_msgs::UploadFilesGoal & goal, std::shared_ptr<SimpleActionClientT> & rosbag_uploader_action_client, int & result_code, const double time_out_in_seconds)
  {
    rosbag_uploader_action_client->sendGoal(goal);
    bool finished_before_timeout = rosbag_uploader_action_client->waitForResult(ros::Duration(time_out_in_seconds));
    if (!finished_before_timeout) {
      AWS_LOG_WARN(__func__, "Uploading rosbags to S3 did not finish before the time out.");
      return UPLOADING_TIMED_OUT;
    }
    auto result = rosbag_uploader_action_client->getResult();
    AWS_LOG_INFO(__func__, "Uploading rosbags to S3 finished with an code: %d", result->result_code);
    result_code = result->result_code;
    return SUCCESS;
  }

  static recorder_msgs::RollingRecorderResult GenerateResult(uint stage, std::string message)
  {
    recorder_msgs::RollingRecorderResult recording_result;
    recorder_msgs::RecorderResult t_recording_result;
    t_recording_result.result = stage;
    t_recording_result.message = std::move(message);
    recording_result.result = t_recording_result;
    return recording_result;
  }

  static recorder_msgs::RollingRecorderFeedback GenerateFeedback(uint stage)
  {
    recorder_msgs::RollingRecorderFeedback record_rosbag_action_feedback;
    record_rosbag_action_feedback.started = ros::Time::now();
    recorder_msgs::RecorderStatus recording_status;
    recording_status.stage = stage;
    record_rosbag_action_feedback.status = recording_status;
    return record_rosbag_action_feedback;
  }
};

}  // namespace Rosbag
}  // namespace Aws
