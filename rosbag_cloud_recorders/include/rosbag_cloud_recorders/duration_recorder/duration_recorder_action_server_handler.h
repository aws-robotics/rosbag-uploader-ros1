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
  static void DurationRecorderStart(Utils::RosbagRecorder& rosbag_recorder, T& goal_handle)
  {
    if (rosbag_recorder.IsActive()) {
      goal_handle.setRejected();
      return;
    }
    goal_handle.setAccepted();
    rosbag_recorder.Run([&] {goal_handle.setRejected();});
  }

  static void CancelDurationRecorder(T& goal_handle)
  {
    goal_handle.setCanceled();
  }
};

}  // namespace Rosbag
}  // namespace Aws
