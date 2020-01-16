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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <aws/core/Aws.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <recorder_msgs/RollingRecorderAction.h>

using namespace Aws::Rosbag;
using RollingRecorderActionClient = actionlib::ActionClient<recorder_msgs::RollingRecorderAction>;

using ::testing::Return;
using ::testing::_;
using ::testing::ContainerEq;
using ::testing::Invoke;

class RollingRecorderTest : public ::testing::Test
{
protected:
  recorder_msgs::RollingRecorderGoal goal;
  ros::AsyncSpinner executor;
  ros::NodeHandle nh;
  RollingRecorderActionClient action_client;
  RollingRecorderActionClient::GoalHandle goal_handle;

  void TearDown() override
  {
    executor.stop();
  }
public:
  RollingRecorderTest():
    executor(0),
    nh("~"),
    action_client(nh, "RosbagRollingRecord")
  {
    executor.start();
  }
};

TEST_F(RollingRecorderTest, TestConstructor)
{
  ros::Duration max_record_time(5);
  ros::Duration bag_rollover_time(5);
  std::string write_directory("~/.ros/rosbag_uploader/");

  {
    Aws::Rosbag::RollingRecorder rolling_recorder(bag_rollover_time, max_record_time, write_directory);
  }

  {
    Aws::Rosbag::RollingRecorder rolling_recorder(bag_rollover_time, max_record_time);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_rolling_recorder_node");
  auto result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
