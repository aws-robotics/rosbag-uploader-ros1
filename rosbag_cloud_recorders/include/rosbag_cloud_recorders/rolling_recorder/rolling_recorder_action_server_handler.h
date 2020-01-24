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
  static void RollingRecorderRosbagUpload(T& goal_handle, const std::string& write_directory, const ros::Duration& bag_rollover_time)
  {
    goal_handle.setRejected();
    //TODO(abbyxu): to add logic to send bags to s3
    ros::Time time_of_goal_received(ros::Time::now());
    std::vector<std::string> ros_bags_to_upload = GetRosbagsToUpload(write_directory, bag_rollover_time, time_of_goal_received);
    //TODO(abbyxu): remove in goalcallback implementation
    ros_bags_to_upload.emplace_back("Test");
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
};

}  // namespace Rosbag
}  // namespace Aws
