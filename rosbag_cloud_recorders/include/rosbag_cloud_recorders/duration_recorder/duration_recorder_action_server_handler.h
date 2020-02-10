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

#include <thread>

#include <ros/ros.h>

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
public:
  static void HandleDurationRecorderUploadResult(
    GoalHandleT goal_handle,
    const actionlib::SimpleClientGoalState& end_state)//,
    //file_uploader_msgs::UploadFilesResultConstPtr /*upload_result*/)
  {
    recorder_msgs::DurationRecorderResult result;
    std::string msg;
    if (end_state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
      msg = "Upload failed with message: " + end_state.getText();
      result.result.result = recorder_msgs::RecorderResult::DEPENDENCY_FAILURE;
      goal_handle.setAborted(result, msg);
    } else {
      result.result.result = recorder_msgs::RecorderResult::SUCCESS;
      msg = "Upload Succeeded";
      goal_handle.setSucceeded(result, msg);
    }
  }

  static void DurationRecorderStart(
    Utils::RosbagRecorder<Utils::Recorder>& rosbag_recorder,
    const DurationRecorderOptions& duration_recorder_options,
    UploadClientT& upload_client,
    GoalHandleT& goal_handle)
  {
    // Used for logging in lambda function
    static auto current_function = __func__;

    AWS_LOG_INFO(__func__, "Goal received");
    if (rosbag_recorder.IsActive()) {
      AWS_LOG_INFO(__func__, "Rejecting goal since recorder already active");
      goal_handle.setRejected();
      return;
    }
    AWS_LOG_INFO(__func__, "Accepted new goal");
    goal_handle.setAccepted();
    static ros::Time time_of_goal_received = ros::Time::now();
    const auto & goal = goal_handle.getGoal();
    Utils::RecorderOptions options;
    // TODO(prasadra): handle invalid input.
    options.record_all = false;
    options.max_duration = goal->duration;
    options.topics = goal->topics_to_record;
    options.prefix = duration_recorder_options.write_directory;
    rosbag_recorder.Run(
      options,
      [goal_handle]() mutable
      {
        recorder_msgs::DurationRecorderFeedback feedback;
        feedback.started = time_of_goal_received;
        recorder_msgs::RecorderStatus recording_status;
        recording_status.stage = recorder_msgs::RecorderStatus::RECORDING;
        feedback.status = recording_status;
        goal_handle.publishFeedback(feedback);
      },
      [goal_handle, duration_recorder_options, &upload_client](int exit_code) mutable
      {
        recorder_msgs::DurationRecorderResult result;
        if (exit_code != 0) {
          AWS_LOG_INFO(current_function, "Recorder finished with non zero exit code, aborting goal");
          goal_handle.setAborted(result, "Rosbag recorder encountered errors");
          return;
        }
        AWS_LOG_INFO(current_function, "Recording finished");
        std::vector<std::string> ros_bags_to_upload = Utils::GetRosbagsToUpload(duration_recorder_options.write_directory,
              [](rosbag::View& rosbag) -> bool
              {
                return time_of_goal_received < rosbag.getBeginTime();
              }
          );
        auto goal = Utils::ConstructRosBagUploaderGoal(goal_handle.getGoal()->destination, ros_bags_to_upload);
        upload_client.sendGoal(goal);
        upload_client.waitForResult();
        HandleDurationRecorderUploadResult(goal_handle, upload_client.getState());//, upload_client.getResult());
      }
    );
  }

  static void CancelDurationRecorder(GoalHandleT& goal_handle)
  {
    goal_handle.setCanceled();
  }
};

}  // namespace Rosbag
}  // namespace Aws
