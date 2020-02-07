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
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder_action_server_handler.h>
#include <rosbag_cloud_recorders/utils/file_utils.h>
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
  static void RollingRecorderRosbagUpload(T& goal_handle, const std::string& write_directory, const ros::Duration& /*bag_rollover_time*/)
  {
    goal_handle.setRejected();
    //TODO(abbyxu): to add logic to send bags to s3
    ros::Time time_of_goal_received(ros::Time::now());
    std::vector<std::string> ros_bags_to_upload = Utils::GetRosbagsToUpload(write_directory,
      [time_of_goal_received](rosbag::View& rosbag) -> bool
      {
        return time_of_goal_received >= rosbag.getBeginTime();
      }
    );
    //TODO(abbyxu): remove in goalcallback implementation
    ros_bags_to_upload.emplace_back("Test");
  }

  static void CancelRollingRecorderRosbagUpload(T& goal_handle)
  {
    goal_handle.setCanceled();
  }
};

}  // namespace Rosbag
}  // namespace Aws
