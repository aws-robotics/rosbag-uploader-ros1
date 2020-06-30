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
#include <boost/function_types/parameter_types.hpp>
#include <boost/typeof/std/utility.hpp>

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

template<typename RecorderFeedbackT, typename RecorderStatusT>
void GenerateFeedback(
  uint8_t stage,
  ros::Time time_of_feedback,
  RecorderFeedbackT& recorder_feedback,
  RecorderStatusT& recording_status)
{
  recorder_feedback.started = time_of_feedback;
  recording_status.stage = stage;
  recorder_feedback.status = recording_status;
}

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

  // Getting the Feedback type from the type of the first arugment to GoalHandleT::publishFeedback()
  // because GoalHandleT::Feedback is a private typedef
  using FuncType = decltype(&GoalHandleT::publishFeedback);
  using FuncArgsType = typename boost::function_types::parameter_types<FuncType>;
  using ArgCrefType = typename boost::mpl::at_c<FuncArgsType, 1>::type;
  using ArgConstType = typename boost::remove_reference<ArgCrefType>::type;
  using ArgType = typename boost::remove_const<ArgConstType>::type;
  ArgType recorder_feedback;
  recorder_msgs::RecorderStatus recording_status;
  Utils::GenerateFeedback(
    recorder_msgs::RecorderStatus::UPLOADING,
    ros::Time::now(),
    recorder_feedback,
    recording_status);
  goal_handle.publishFeedback(recorder_feedback);
  bool upload_finished = true;
  if (upload_time_out > 0) {
    upload_finished = upload_client.waitForResult(ros::Duration(upload_time_out));
  } else {
    upload_finished = upload_client.waitForResult();
  }

  return upload_finished;
}

template<typename RecorderResultT>
void GenerateResult(uint8_t stage, std::string message, RecorderResultT& recorder_result)
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
  // Getting the Feedback type from the type of the first arugment to GoalHandleT::publishFeedback()
  // because GoalHandleT::Feedback is a private typedef
  using FuncType = decltype(&GoalHandleT::publishFeedback);
  using FuncArgsType = typename boost::function_types::parameter_types<FuncType>;
  using ArgCrefType = typename boost::mpl::at_c<FuncArgsType, 1>::type;
  using ArgConstType = typename boost::remove_reference<ArgCrefType>::type;
  using ArgType = typename boost::remove_const<ArgConstType>::type;
  ArgType recorder_feedback;
  recorder_msgs::RecorderStatus recording_status;
  Utils::GenerateFeedback(
    recorder_msgs::RecorderStatus::COMPLETE,
    ros::Time::now(),
    recorder_feedback,
    recording_status);
  goal_handle.publishFeedback(recorder_feedback);

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

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
