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
#include <ros/ros.h>
#include <ros/spinner.h>
#include <rosbag/recorder.h>

#include <recorder_msgs/DurationRecorderAction.h>
#include <recorder_common_error_codes.h>

namespace Aws
{
namespace Rosbag
{

typedef actionlib::ActionServer<recorder_msgs::DurationRecorderAction> DurationRecorderActionServer;

/**
 *  Duration recorder is a node that responds to actions to record rosbag files
 */
class DurationRecorder
{
private:
    ros::NodeHandle node_handle_;
    DurationRecorderActionServer action_server_;
    std::unique_ptr<rosbag::Recorder> rosbag_recorder_;

    void GoalCallBack(DurationRecorderActionServer::GoalHandle goal);
    void CancelGoalCallBack(DurationRecorderActionServer::GoalHandle goal);

public:
    DurationRecorder(const std::vector<std::string> & topics, const ros::Duration & max_duration);
    ~DurationRecorder() = default;

    /**
    * @brief delete a rosbag file that has alreaady been uploaded to an Amazon S3 bucket
    *
    * Delete file at rosbag_file_path.
    *
    * @param rosbag_file_path path to the rosbag file to be deleted
    * @return error code, SUCCESS if the file is sucessfully deleted
    */
    Aws::Rosbag::RecorderErrorCode DeleteUploadedRosbag(const std::string & rosbag_file_path);
};

}  // namespace Rosbag
}  // namespace Aws
