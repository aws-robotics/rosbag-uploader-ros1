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
#include <recorder_msgs/RollingRecorderAction.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <boost/filesystem.hpp>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace Aws{
namespace Rosbag{

using RollingRecorderActionServer = actionlib::ActionServer<recorder_msgs::RollingRecorderAction>;

template<typename T>
class RollingRecorderActionServerHandler
{
public:
  static void RollingRecorderRosbagUpload(T& goal_handle,
                                          const std::string& write_directory,
                                          const ros::Duration& bag_rollover_time,
                                          RollingRecorderActionServer::GoalHandle * current_goal_handle,
                                          std::unique_ptr<UploadFilesActionSimpleClient> & rosbag_uploader_action_client)
  {
    AWS_LOG_INFO(__func__, "A new goal has been recieved by the goal action server");
    recorder_msgs::RollingRecorderResult recording_result;
    recorder_msgs::RecorderResult t_recording_result;

    //  Check if rolling recorder action server is currently processing a goal
    if(current_goal_handle) {
      //  Check if new goal is the same goal as the current one being processed
      if (*current_goal_handle == goal_handle) {
        AWS_LOG_INFO(__func__, "New goal recieved by the rolling recorder action server is the same goal the server is processing.");
      } else {
        AWS_LOG_WARN(__func__, "Rejecting new goal due to rolling recorder action recorder is processing a goal.");
        GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::INTERNAL_ERROR, "Rejected because server is currently processing another goal.");
        goal_handle.setRejected(recording_result, "");
      }
      return;
    }
    ros::Time time_of_goal_received(ros::Time::now());
    boost::shared_ptr<const recorder_msgs::RollingRecorderGoal> goal = goal_handle.getGoal();

    //  Check if goal is valid
    if (!ValidateGoal(goal)) {
      AWS_LOG_ERROR(__func__, "Goal was not valid, rejecting goal...");
      GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::INVALID_INPUT, "Goal was not valid.");
      goal_handle.setRejected(recording_result, "");
      return;
    }

    AWS_LOG_INFO(__func__, "Trying to fiullfill current goal...")

    std::vector<std::string> ros_bags_to_upload = GetRosbagsToUpload(write_directory, bag_rollover_time, time_of_goal_received);
    if (ros_bags_to_upload.empty()) {
      AWS_LOG_ERROR(__func__, "Did not find any ros bags to upload, rejecting goal...");
      GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::INVALID_INPUT, "Goal was valid but no upload action could be taken.");
      goal_handle.setRejected(recording_result, "");
    }

    goal_handle.setAccepted();
    // SetCurrentGoalHandle(goal_handle);  //  Take in new Goal
    recorder_msgs::RollingRecorderFeedback record_rosbag_action_feedback;
    GenerateFeedback(record_rosbag_action_feedback, recorder_msgs::RecorderStatus::RECORDING);
    goal_handle.publishFeedback(record_rosbag_action_feedback);

    AWS_LOG_INFO(__func__, "Recoding goal completed with a status: Succeeded.");

    GenerateFeedback(record_rosbag_action_feedback, recorder_msgs::RecorderStatus::UPLOADING);
    goal_handle.publishFeedback(record_rosbag_action_feedback);

    rosbag_uploader_action_client->waitForServer(ros::Duration(10, 0));
    if (!rosbag_uploader_action_client->isServerConnected()) {
      AWS_LOG_WARN(__func__, "Not able to connect to file uploader action server, rosbags uploading failed to complete.");
      GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::INTERNAL_ERROR, "Not able to connect to file uploader action server, rosbags uploading failed to complete.");
      goal_handle.setAborted(recording_result, "");
    } else {
      file_uploader_msgs::UploadFilesGoal file_uploader_goal = ConstructRosBagUploaderGoal(goal->destination, ros_bags_to_upload);
      RecorderErrorCode upload_status = SendRosBagUploaderGoal(file_uploader_goal, rosbag_uploader_action_client);
      if (SUCCESS == upload_status) {
        GenerateResult(recording_result, t_recording_result, recorder_msgs::RecorderResult::SUCCESS, "Rolling recording and rosbags uploading were completed successfully.");
        goal_handle.setSucceeded(recording_result, "");
      } else {
        GenerateResult(recording_result, t_recording_result, upload_status, "Rolling recording succeeded, however, rosbags uploading failed to complete.");
        goal_handle.setAborted(recording_result, "");
      }
    }
    // ReleaseCurrentGoalHandle();
  }

  static void CancelRollingRecorderRosbagUpload(T& goal_handle)
  {
    goal_handle.setCanceled();
  }

  static std::vector<std::string> GetRosbagsToUpload(const std::string& write_directory, const ros::Duration& bag_rollover_time, ros::Time time_of_goal_received)
  {
    std::vector<std::string> ros_bags_to_upload;
    boost::filesystem::path ros_bag_write_path(write_directory);
    for (auto i = boost::filesystem::directory_iterator(ros_bag_write_path); i != boost::filesystem::directory_iterator(); i++) {
      if (!boost::filesystem::is_directory(i->path())) {  // eliminate directories in a list
        if (i->path().extension().string() == ".bag") {
          ros_bags_to_upload.push_back(write_directory + i->path().filename().string());
          AWS_LOG_INFO(__func__, "Adding Rosbag named: [%s] to list of bag file to upload.", i->path().filename().string().c_str());
        }
        if (i->path().extension().string() == ".active") {
          bag_rollover_time.sleep();
          std::string bag_file_name_wo_active_ext = std::string(i->path().filename().string()).erase(i->path().filename().string().size() - 7);
          rosbag::Bag ros_bag;
          ros_bag.open(write_directory + bag_file_name_wo_active_ext);

          rosbag::View view_rosbag(ros_bag);
          if (time_of_goal_received >= view_rosbag.getBeginTime()) {
            ros_bags_to_upload.push_back(write_directory + bag_file_name_wo_active_ext);
          }
        }
      }
    }
    return ros_bags_to_upload;
  }

private:
  static bool ValidateGoal(boost::shared_ptr<const recorder_msgs::RollingRecorderGoal> goal)
  {
    if (goal->destination.empty()) {
      return false;
    }
    return true;
  }

  static file_uploader_msgs::UploadFilesGoal ConstructRosBagUploaderGoal(std::string destination, std::vector<std::string> & ros_bags_to_upload)
  {
    AWS_LOG_INFO(__func__, "Constructing Uploader Goal.");
    file_uploader_msgs::UploadFilesGoal file_uploader_goal;
    file_uploader_goal.files = ros_bags_to_upload;
    file_uploader_goal.upload_location = destination;
    return file_uploader_goal;
  }

  static RecorderErrorCode SendRosBagUploaderGoal(const file_uploader_msgs::UploadFilesGoal & goal, std::unique_ptr<UploadFilesActionSimpleClient> & rosbag_uploader_action_client)
  {
    AWS_LOG_INFO(__func__, "Sending rosbag uploader goal to uploader action server.");
    // TODO(abbyxu): wait for rosbag upload return recorder_msgs
    rosbag_uploader_action_client->sendGoal(goal);
    return SUCCESS;
  }

  static void ReleaseCurrentGoalHandle(RollingRecorderActionServer::GoalHandle & current_goal_handle)
  {
    current_goal_handle = nullptr;
  }

  static void SetCurrentGoalHandle(RollingRecorderActionServer::GoalHandle & current_goal_handle, RollingRecorderActionServer::GoalHandle & new_goal_handle)
  {
    current_goal_handle = &new_goal_handle;
  }

  static void GenerateResult(recorder_msgs::RollingRecorderResult & recording_result, recorder_msgs::RecorderResult & t_recording_result, uint stage, std::string message)
  {
    t_recording_result.result = stage;
    t_recording_result.message = message;
    recording_result.result = t_recording_result;
  }

  static void GenerateFeedback(recorder_msgs::RollingRecorderFeedback & record_rosbag_action_feedback, uint stage)
  {
    record_rosbag_action_feedback.started = ros::Time::now();
    recorder_msgs::RecorderStatus recording_status;
    recording_status.stage = stage;
    record_rosbag_action_feedback.status = recording_status;
  }
};

}  // namespace Rosbag
}  // namespace Aws
