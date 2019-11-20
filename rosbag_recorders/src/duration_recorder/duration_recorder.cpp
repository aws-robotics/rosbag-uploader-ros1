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

#include <recorder_msgs/DurationRecorderAction.h>
#include <recorder_common_error_codes.h>
#include <duration_recorder/duration_recorder.h>

namespace Aws
{
namespace Rosbag
{

DurationRecorder::DurationRecorder(const std::vector<std::string> & topics, const ros::Duration & max_duration) :
    node_handle_("~"),
    action_server_(node_handle_, "RosbagDurationRecord", false)
{
}

void DurationRecorder::GoalCallBack(DurationRecorderActionServer::GoalHandle goal_handle)
{
}

void DurationRecorder::CancelGoalCallBack(DurationRecorderActionServer::GoalHandle goal_handle)
{
}

Aws::Rosbag::RecorderErrorCode DurationRecorder::DeleteUploadedRosbag(const std::string & rosbag_file_path)
{
}

}  // namespace Rosbag
}  // namespace Aws
