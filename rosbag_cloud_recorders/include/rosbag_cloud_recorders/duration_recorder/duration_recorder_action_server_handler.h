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

#include <actionlib/server/action_server.h>
#include <recorder_msgs/DurationRecorderAction.h>

#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>

namespace Aws{
namespace Rosbag{

using DurationRecorderActionServer = actionlib::ActionServer<recorder_msgs::DurationRecorderAction>;

template<typename T>
class DurationRecorderActionServerHandler
{
public:
  static void DurationRecorderStart(Utils::RosbagRecorder<rosbag::Recorder>& rosbag_recorder, T& goal_handle)
  {
    if (rosbag_recorder.IsActive()) {
      goal_handle.setRejected();
      return;
    }
    goal_handle.setAccepted();
    auto goal = goal_handle.getGoal();
    rosbag::RecorderOptions options;
    // TODO(prasadra): handle invalid input.
    options.record_all = false;
    options.max_duration = goal->duration;
    options.topics = goal->topics_to_record;
    rosbag_recorder.Run(
      options,
      [&]()
      {
        recorder_msgs::DurationRecorderFeedback feedback;
        feedback.started = ros::Time::now();
        recorder_msgs::RecorderStatus recording_status;
        recording_status.stage = recorder_msgs::RecorderStatus::RECORDING;
        feedback.status = recording_status;
        goal_handle.publishFeedback(feedback);
      },
      [&]()
      {
        // TODO(prasadra): Implement integration with s3_file _ploader;
        recorder_msgs::DurationRecorderResult result;
        goal_handle.setSucceeded(result, "");
      }
    );
  }

  static void CancelDurationRecorder(T& goal_handle)
  {
    goal_handle.setCanceled();
  }
};

}  // namespace Rosbag
}  // namespace Aws
