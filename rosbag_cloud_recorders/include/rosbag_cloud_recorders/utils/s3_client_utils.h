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
#include <actionlib/client/simple_action_client.h>

#include <file_uploader_msgs/UploadFilesAction.h>

#include <rosbag_cloud_recorders/recorder_common_error_codes.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

file_uploader_msgs::UploadFilesGoal ConstructRosBagUploaderGoal(std::string destination,
  std::vector<std::string> & ros_bags_to_upload);

template<typename GoalHandleT, typename UploadClientT>
bool UploadFiles(
  GoalHandleT& goal_handle,
  const double upload_time_out,
  UploadClientT& upload_client,
  std::vector<std::string>& ros_bags_to_upload)
{
  AWS_LOG_INFO(__func__, "Uploading Files.");
  auto goal = Utils::ConstructRosBagUploaderGoal(goal_handle.getGoal()->destination, ros_bags_to_upload);
  upload_client.sendGoal(goal);
  bool upload_finished = true;
  if (upload_time_out > 0) {
    upload_finished = upload_client.waitForResult(ros::Duration(upload_time_out));
  } else {
    upload_finished = upload_client.waitForResult();
  }
  return upload_finished;
}

template<typename UploadClientT>
bool UploaderSeverConnected(const double server_time_out, UploadClientT& upload_client)
{
  return upload_client.waitForServer(ros::Duration(server_time_out));
}

template<typename RecorderResultT>
void GenerateResult(uint stage, std::string message, RecorderResultT& recorder_result)
{
  recorder_result.result.result = stage;
  recorder_result.result.message = std::move(message);
}

template<typename GoalHandleT, typename SimpleClientGoalStateT, typename RecorderResultT>
void HandleRecorderUploadResult(
  GoalHandleT& goal_handle,
  const SimpleClientGoalStateT& end_state,
  bool upload_finished,
  RecorderResultT& recorder_result)
{
  std::string msg;
  if (!upload_finished) {
    msg = "File Uploader timed out.";
    GenerateResult(recorder_msgs::RecorderResult::UPLOADER_TIMEOUT, msg, recorder_result);
    goal_handle.setAborted(recorder_result, msg);
    AWS_LOG_WARN(__func__, msg.c_str());
    return;
  }
  if (end_state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
    msg = "Upload failed with message: " + end_state.getText();
    GenerateResult(recorder_msgs::RecorderResult::DEPENDENCY_FAILURE, msg, recorder_result);
    goal_handle.setAborted(recorder_result, msg);
    AWS_LOG_ERROR(__func__, msg.c_str());
  } else {
    msg = "Upload Succeeded";
    GenerateResult(recorder_msgs::RecorderResult::SUCCESS, msg, recorder_result);
    goal_handle.setSucceeded(recorder_result, msg);
    AWS_LOG_INFO(__func__, msg.c_str());
  }
}

template<typename RecorderFeedbackT, typename RecorderStatusT>
void GenerateFeedback(
  uint stage,
  ros::Time time_of_goal_received,
  RecorderFeedbackT& recorder_feedback,
  RecorderStatusT& recording_status)
{
  recorder_feedback.started = time_of_goal_received;
  recording_status.stage = stage;
  recorder_feedback.status = recording_status;
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
