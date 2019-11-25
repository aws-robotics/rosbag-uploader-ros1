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

#include <gtest/gtest.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <recorder_msgs/RollingRecorderAction.h>
#include <recorder_common_error_codes.h>
#include <rolling_recorder/rolling_recorder.h>

using namespace Aws::Rosbag;
using namespace rosbag;


typedef actionlib::ActionClient<recorder_msgs::RollingRecorderAction> RollingRecorderActionClient;

class RollingRecorderNodeFixture : public ::testing::Test
{
protected:

  std::shared_ptr<RollingRecorderActionClient> action_client;
  std::shared_ptr<Aws::Rosbag::RollingRecorder> rolling_recorder;

  void SetUp() override
  {
    ros::NodeHandle nh("~");
    action_client = std::make_shared<RollingRecorderActionClient>(nh, "RosbagRollingRecord");
    rolling_recorder = std::make_shared<Aws::Rosbag::RollingRecorder>();
  }
};

TEST_F(RollingRecorderNodeFixture, TestActionReceivedbyActionServer)
{
  ros::AsyncSpinner executor(0);
  executor.start();
  bool message_received = false;
  // Wait 10 seconds for server to start
  ASSERT_TRUE(action_client->waitForActionServerToStart(ros::Duration(10, 0)));
  auto transition_call_back = [&message_received](RollingRecorderActionClient::GoalHandle goal_handle) {
    EXPECT_EQ(goal_handle.getTerminalState().state_, actionlib::TerminalState::StateEnum::REJECTED);
    message_received = true;
  };
  recorder_msgs::RollingRecorderGoal goal;
  auto gh = action_client->sendGoal(goal, transition_call_back);
  ros::Duration(1, 0).sleep();
  ASSERT_TRUE(message_received);
  executor.stop();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_rolling_recorder_node");
  auto result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
