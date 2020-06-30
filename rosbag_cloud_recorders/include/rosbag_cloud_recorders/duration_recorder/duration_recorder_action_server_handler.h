/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <sstream>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/duration.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/action_server.h>

#include <file_uploader_msgs/UploadFilesAction.h>
#include <recorder_msgs/DurationRecorderAction.h>

#include <aws/core/utils/logging/LogMacros.h>

#include <rosbag_cloud_recorders/duration_recorder/duration_recorder.h>
#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>
#include <rosbag_cloud_recorders/utils/file_utils.h>
#include <rosbag_cloud_recorders/utils/s3_client_utils.h>

namespace Aws{
namespace Rosbag{

using DurationRecorderActionServer = actionlib::ActionServer<recorder_msgs::DurationRecorderAction>;



template<typename GoalHandleT, typename UploadClientT>
class DurationRecorderActionServerHandler
{
private:
  static bool ValidateGoal(GoalHandleT& goal_handle)
  {
    const auto & goal = goal_handle.getGoal();
    if (goal->duration <= ros::Duration(0) || goal->duration > ros::DURATION_MAX) {
      std::stringstream msg;
      msg << "Goal rejected. Invalid record duration given: " << goal->duration;
      recorder_msgs::DurationRecorderResult result;
      Utils::GenerateResult(recorder_msgs::RecorderResult::INVALID_INPUT, msg.str(), result);
      goal_handle.setRejected(result, result.result.message);
      AWS_LOG_INFO(__func__, result.result.message.c_str());
      return false;
    }
    return true;
  }
public:
  static void DurationRecorderStart(
    Utils::RosbagRecorder<Utils::Recorder>& rosbag_recorder,
    const DurationRecorderOptions& duration_recorder_options,
    UploadClientT& upload_client,
    GoalHandleT& goal_handle)
  {
    // Used for logging in lambda function
    static auto current_function = __func__;
    ros::Time time_of_goal_received = ros::Time::now();

    AWS_LOG_INFO(__func__, "Goal received");
    if (rosbag_recorder.IsActive()) {
      const std::string msg = "Rejecting goal since recorder already active";
      recorder_msgs::DurationRecorderResult result;
      Utils::GenerateResult(recorder_msgs::RecorderResult::INTERNAL_ERROR, msg, result);
      goal_handle.setRejected(result, msg);
      AWS_LOG_INFO(__func__, msg.c_str());
      return;
    }

    if (!ValidateGoal(goal_handle)) {
      // Goal was invalid and rejected
      return;
    }

    const auto & goal = goal_handle.getGoal();
    Utils::RecorderOptions options;
    options.max_duration = goal->duration;
    options.min_space = 1024 * 1024 * duration_recorder_options.min_free_space_mib; // mebibytes to bytes
    options.min_space_str = std::to_string(duration_recorder_options.min_free_space_mib) + 'M';
    if (goal->topics_to_record.empty()) {
      options.record_all = true;
    } else {
      options.record_all = false;
      options.topics = goal->topics_to_record;
    }
    options.prefix = duration_recorder_options.write_directory;

    auto run_result = rosbag_recorder.Run(
      options,
      [goal_handle, time_of_goal_received]() mutable
      {
        goal_handle.setAccepted();
        AWS_LOG_INFO(current_function, "Goal accepted");

        recorder_msgs::DurationRecorderFeedback recorder_feedback;
        recorder_msgs::RecorderStatus recording_status;
        Utils::GenerateFeedback(
          recorder_msgs::RecorderStatus::RECORDING,
          time_of_goal_received,
          recorder_feedback,
          recording_status);
        goal_handle.publishFeedback(recorder_feedback);
      },
      [goal_handle, duration_recorder_options, time_of_goal_received, &upload_client](int exit_code) mutable
      {
        recorder_msgs::DurationRecorderResult result;
        if (exit_code != 0) {
          const std::string msg = "Rosbag recorder encountered errors.";
          Utils::GenerateResult(recorder_msgs::RecorderResult::INTERNAL_ERROR, msg, result);
          goal_handle.setAborted(result, msg);
          AWS_LOG_INFO(current_function, "Recorder finished with non zero exit code, aborting goal");
          return;
        }

        recorder_msgs::DurationRecorderFeedback recorder_feedback;
        recorder_msgs::RecorderStatus recording_status;
        Utils::GenerateFeedback(
          recorder_msgs::RecorderStatus::PREPARING_UPLOAD,
          ros::Time::now(),
          recorder_feedback,
          recording_status);
        goal_handle.publishFeedback(recorder_feedback);
        std::vector<std::string> ros_bags_to_upload = Utils::GetRosbagsToUpload(duration_recorder_options.write_directory,
          [time_of_goal_received](rosbag::View& rosbag) -> bool
          {
            // Select bags that were recorded during this duration recorder goal.
            // Older bags may be left over from previous runs of the recorder.
            return time_of_goal_received < rosbag.getBeginTime();
          }
        );

        bool upload_finished = Utils::UploadFiles(goal_handle, duration_recorder_options.upload_timeout_s, upload_client, ros_bags_to_upload);

        Utils::HandleRecorderUploadResult(goal_handle, upload_client.getState(), upload_finished, result);

        if (duration_recorder_options.delete_bags_after_upload) {
          Utils::GenerateFeedback(
            recorder_msgs::RecorderStatus::CLEANUP,
            ros::Time::now(),
            recorder_feedback,
            recording_status);
          goal_handle.publishFeedback(recorder_feedback);
          for (const std::string & bag_file_name : upload_client.getResult()->files_uploaded) {
            AWS_LOG_INFO(current_function, "Bag file named: %s was uploaded to S3 and is now being deleted locally.", bag_file_name.c_str());
            Utils::DeleteFile(bag_file_name);
          }
        }
      }
    );

    if (Utils::RosbagRecorderRunResult::SKIPPED == run_result) {
      recorder_msgs::DurationRecorderResult result;
      result.result.result = recorder_msgs::RecorderResult::INTERNAL_ERROR;
      goal_handle.setRejected(result, "Rejecting result, DurationRecorder already handling goal.");
    }
  }

  static void CancelDurationRecorder(GoalHandleT& goal_handle)
  {
    goal_handle.setCanceled();
  }
};

}  // namespace Rosbag
}  // namespace Aws
