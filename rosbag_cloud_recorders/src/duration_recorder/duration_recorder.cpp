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

#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <ros/ros.h>
#include <rosbag/recorder.h>
#include <aws/core/utils/logging/LogMacros.h>

#include <recorder_msgs/DurationRecorderAction.h>
#include <recorder_common_error_codes.h>
#include <duration_recorder/duration_recorder.h>


namespace Aws
{
namespace Rosbag
{

DurationRecorder::DurationRecorder() :
    node_handle_("~"),
    action_server_(node_handle_, "RosbagDurationRecord", false)
{
  rosbag_recorder_ = nullptr;
  current_goal_handle_ = nullptr;
  action_server_.registerGoalCallback(
      boost::bind(&DurationRecorder::GoalCallBack, this, _1));
  action_server_.registerCancelCallback(
      boost::bind(&DurationRecorder::CancelGoalCallBack, this, _1));
  action_server_.start();
}

void DurationRecorder::GoalCallBack(DurationRecorderActionServer::GoalHandle goal_handle)
{
  AWS_LOG_INFO(__func__, "A new goal has been recieved by the goal action server");

  recorder_msgs::DurationRecorderResult recording_result;
  recorder_msgs::RecorderResult t_recording_result;
  if(current_goal_handle_) {
    if (*current_goal_handle_ == goal_handle) {
      AWS_LOG_INFO(__func__, "New goal recieved by the goal action server is the same goal as the goal duration action server is processing.");
    } else {
      AWS_LOG_WARN(__func__, "Reject new goal due to duration recorder action recorder is processing another goal.");
      GenerateResult(recording_result, t_recording_result, 2, "Rejected because currently processing another goal.");
      goal_handle.setRejected();
    }
    return;
  }

  goal_handle.setAccepted();
  SetCurrentGoalHandle(goal_handle);

  AWS_LOG_INFO(__func__, "Publishing feedback...");
  recorder_msgs::DurationRecorderFeedback record_rosbag_action_feedback;
  GenerateFeedback(record_rosbag_action_feedback, 0);
  goal_handle.publishFeedback(record_rosbag_action_feedback);
  AWS_LOG_INFO(__func__, "Starting duration recorder...")
  boost::shared_ptr<const recorder_msgs::DurationRecorderGoal> goal = goal_handle.getGoal();

  //TODO(abbyxu): add retry logic
  if (!StartDurationRecorder(goal)) {
    AWS_LOG_ERROR(__func__, "Duration recording finished with a status: Failed");
    GenerateResult(recording_result, t_recording_result, 2, "Duration recording failed.");
    goal_handle.setAborted(recording_result, "");
    ReleaseCurrentGoalHandle();
    return;
  }
  AWS_LOG_INFO(__func__, "Duration recording finished with a status: Succeeded");
  GenerateResult(recording_result, t_recording_result, 0, "Duration recording succeeded.");
  goal_handle.setSucceeded(recording_result, "");
  ReleaseCurrentGoalHandle();
}

void DurationRecorder::GenerateResult(recorder_msgs::DurationRecorderResult & recording_result, recorder_msgs::RecorderResult & t_recording_result, uint stage, std::string message)
{
  t_recording_result.result = stage;
  t_recording_result.message = message;
  recording_result.result = t_recording_result;
}

void DurationRecorder::GenerateFeedback(recorder_msgs::DurationRecorderFeedback & record_rosbag_action_feedback, uint stage)
{
  record_rosbag_action_feedback.started = ros::Time::now();
  recorder_msgs::RecorderStatus recording_status;
  recording_status.stage = stage;
  record_rosbag_action_feedback.status = recording_status;
}

void DurationRecorder::ConfigureRecorderOptions(boost::shared_ptr<const recorder_msgs::DurationRecorderGoal> goal, rosbag::RecorderOptions recorder_options)
{
  AWS_LOG_INFO(__func__, "Starting configuring duration recorder...");
  recorder_options.topics = goal->topics_to_record;
  recorder_options.prefix = goal->destination;
  recorder_options.max_duration = goal->duration;
  AWS_LOG_INFO(__func__, "Finished configuring duration recorder...");
}

int DurationRecorder::StartDurationRecorder(boost::shared_ptr<const recorder_msgs::DurationRecorderGoal> goal)
{
  if (!rosbag_recorder_) {
    rosbag::RecorderOptions recorder_options = {};
    ConfigureRecorderOptions(goal, recorder_options);
    AWS_LOG_INFO(__func__, "Initializing duration recorder...");
    rosbag_recorder_ = std::make_unique<rosbag::Recorder>(recorder_options);
    AWS_LOG_INFO(__func__, "Finished initializing duration recorder...");
  }
  AWS_LOG_INFO(__func__, "Recording rosbag...");
  return rosbag_recorder_->run();
}

void DurationRecorder::SetCurrentGoalHandle(DurationRecorderActionServer::GoalHandle & new_goal_handle)
{
  current_goal_handle_ = &new_goal_handle;
}

void DurationRecorder::ReleaseCurrentGoalHandle()
{
  current_goal_handle_ = nullptr;
}

void DurationRecorder::CancelGoalCallBack(DurationRecorderActionServer::GoalHandle goal_handle)
{
  (void) goal_handle;
}

}  // namespace Rosbag
}  // namespace Aws
