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

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <rosbag/recorder.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <recorder_msgs/RollingRecorderAction.h>
#include <recorder_common_error_codes.h>
#include <rolling_recorder/rolling_recorder.h>
#include <string>
#include <vector>

namespace Aws
{
namespace Rosbag
{

RollingRecorder::RollingRecorder(
  ros::Duration bag_rollover_time, ros::Duration max_record_time,
  std::vector<std::string> topics_to_record, std::string write_directory) :
  node_handle_("~"),
  action_server_(node_handle_, "RosbagRollingRecord", false),
  rosbag_uploader_action_client_(std::make_unique<UploadFilesActionSimpleClient>("/s3_file_uploader/UploadFiles", true)),
  max_duration_(max_record_time),
  current_goal_handle_(nullptr),
  is_rolling_recorder_running_(false),
  write_directory_(write_directory)
{
  rosbag::RecorderOptions rolling_recorder_options;
  rolling_recorder_options.max_duration = bag_rollover_time;
  rolling_recorder_options.topics = topics_to_record;
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
  ros::Duration bag_rollover_time, ros::Duration max_record_time, std::vector<std::string> topics_to_record)
  : RollingRecorder(bag_rollover_time, max_record_time, topics_to_record, "~/.ros/rosbag_uploader/")
{
}

void RollingRecorder::GoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle)
{
  double current_time_in_sec = ros::Time::now().toSec();
  AWS_LOG_INFO(__func__, "A new goal has been recieved by the goal action server");
  recorder_msgs::RollingRecorderResult recording_result;
  recorder_msgs::RecorderResult t_recording_result;

  //  Check if rolling recorder action server is currently processing a goal
  if(current_goal_handle_) {
    //  Check if new goal is the same goal as the current one being processed
    if (*current_goal_handle_ == goal_handle) {
      AWS_LOG_INFO(__func__, "New goal recieved by the rolling recorder action server is the same goal the server is processing.");
    } else {
      AWS_LOG_WARN(__func__, "Rejecting new goal due to rolling recorder action recorder is processing a goal.");
      GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::INTERNAL_ERROR, "Rejected because server is currently processing another goal.");
      goal_handle.setRejected(recording_result, "");
    }
    return;
  }

  boost::shared_ptr<const recorder_msgs::RollingRecorderGoal> goal = goal_handle.getGoal();

  //  Check if goal is valid
  if (!ValidateGoal(goal, current_time_in_sec)) {
    AWS_LOG_ERROR(__func__, "Goal was not valid, rejecting goal...");
    GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::INVALID_INPUT, "Goal was not valid.");
    goal_handle.setRejected(recording_result, "");
    ReleaseCurrentGoalHandle();
    return;
  }

  //TODO(abbyxu): add retry logic
  //  Check if Rolling Recorder is running
  if (!is_rolling_recorder_running_) {
    AWS_LOG_ERROR(__func__, "Rolling recording did not start.");
    GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::INTERNAL_ERROR, "Rolling recording failed.");
    goal_handle.setAborted(recording_result, "");
    ReleaseCurrentGoalHandle();
    return;
  }

  goal_handle.setAccepted();
  SetCurrentGoalHandle(goal_handle);  //  Take in new Goal

  recorder_msgs::RollingRecorderFeedback record_rosbag_action_feedback;
  GenerateFeedback(record_rosbag_action_feedback, recorder_msgs::RecorderStatus::RECORDING);
  goal_handle.publishFeedback(record_rosbag_action_feedback);
  AWS_LOG_INFO(__func__, "Trying to fiullfill current goal...")
  //  TODO: logic to check bag files between start time and end time
  AWS_LOG_INFO(__func__, "Recoding goal completed with a status: Succeeded.");

  GenerateFeedback(record_rosbag_action_feedback, recorder_msgs::RecorderStatus::UPLOADING);
  goal_handle.publishFeedback(record_rosbag_action_feedback);

  //TODO(abbyxu): set timeout as const and set retry
  rosbag_uploader_action_client_->waitForServer(ros::Duration(10, 0));
  if (!rosbag_uploader_action_client_->isServerConnected()) {
    AWS_LOG_WARN(__func__, "Not able to connect to file uploader action server, rosbags uploading failed to complete.");
    GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::INTERNAL_ERROR, "Not able to connect to file uploader action server, rosbags uploading failed to complete.");
    goal_handle.setAborted(recording_result, "");
  } else {
    file_uploader_msgs::UploadFilesGoal file_uploader_goal = ConstructRosBagUploaderGoal(goal->destination);
    RecorderErrorCode upload_status = SendRosBagUploaderGoal(file_uploader_goal);
    if (SUCCESS == upload_status) {
      GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::SUCCESS, "Rolling recording and rosbags uploading were completed successfully.");
      goal_handle.setSucceeded(recording_result, "");
    } else {
      GenerateResult(recording_result, t_recording_result, upload_status, "Rolling recording succeeded, however, rosbags uploading failed to complete.");
      goal_handle.setAborted(recording_result, "");
    }
  }
  ReleaseCurrentGoalHandle();
  //TODO(abbyxu): add delete logic or update the map
}

RecorderErrorCode RollingRecorder::SendRosBagUploaderGoal(const file_uploader_msgs::UploadFilesGoal & goal)
{
  AWS_LOG_INFO(__func__, "Sending rosbag uploader goal to uploader action server.");
  rosbag_uploader_action_client_->sendGoal(goal);
  return SUCCESS;
}

file_uploader_msgs::UploadFilesGoal RollingRecorder::ConstructRosBagUploaderGoal(std::string destination) const
{
  AWS_LOG_INFO(__func__, "Constructing Uploader Goal.");
  file_uploader_msgs::UploadFilesGoal file_uploader_goal;
  file_uploader_goal.files = GetRosgBagListToUplaod();
  file_uploader_goal.upload_location = destination;
  return file_uploader_goal;
}

bool RollingRecorder::ValidateGoal(boost::shared_ptr<const recorder_msgs::RollingRecorderGoal> goal, double current_time_in_sec)
{
  if (goal->start_time.toSec() > current_time_in_sec || goal->start_time.toSec() < current_time_in_sec - max_duration_.toSec()) {
    return false;
  }

  if (goal->end_time.toSec() <= goal->start_time.toSec() || goal->end_time.toSec() >= current_time_in_sec + max_duration_.toSec()) {
    return false;
  }

  if (goal->destination.empty()) {
    return false;
  }
  return true;
}

std::vector<std::string> RollingRecorder::GetRosgBagListToUplaod() const
{
  std::vector<std::string> rosbag_to_upload;
  return rosbag_to_upload;
}
void RollingRecorder::SetCurrentGoalHandle(RollingRecorderActionServer::GoalHandle & new_goal_handle)
{
  current_goal_handle_ = &new_goal_handle;
}

void RollingRecorder::ReleaseCurrentGoalHandle()
{
  current_goal_handle_ = nullptr;
}

void RollingRecorder::CancelGoalCallBack(RollingRecorderActionServer::GoalHandle goal_handle)
{
  (void) goal_handle;
}

void RollingRecorder::GenerateResult(recorder_msgs::RollingRecorderResult & recording_result, recorder_msgs::RecorderResult & t_recording_result, uint stage, std::string message)
{
  t_recording_result.result = stage;
  t_recording_result.message = message;
  recording_result.result = t_recording_result;
}

void RollingRecorder::GenerateFeedback(recorder_msgs::RollingRecorderFeedback & record_rosbag_action_feedback, uint stage)
{
  record_rosbag_action_feedback.started = ros::Time::now();
  recorder_msgs::RecorderStatus recording_status;
  recording_status.stage = stage;
  record_rosbag_action_feedback.status = recording_status;
}

RecorderErrorCode RollingRecorder::StartRollingRecorder()
{
  return SUCCESS;
}

RecorderErrorCode RollingRecorder::StopRollingRecorder()
{
  return SUCCESS;
}

bool RollingRecorder::IsRollingRecorderActive() const
{
  return is_rolling_recorder_running_;
}

}  // namespace Rosbag
}  // namespace Aws
