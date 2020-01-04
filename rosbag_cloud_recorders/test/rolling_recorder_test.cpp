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
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <recorder_msgs/RollingRecorderAction.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>

using namespace Aws::Rosbag;
using RollingRecorderActionClient = actionlib::ActionClient<recorder_msgs::RollingRecorderAction>;

class RollingRecorderNodeFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle nh("~");
    const ros::Duration max_record_time(5);
    const ros::Duration bag_rollover_time(5);
    std::string write_directory("~/.ros/rosbag_uploader/");
    boost::filesystem::create_directory(write_directory);
    path_ = boost::filesystem::path(write_directory);
    action_client_ = std::make_shared<RollingRecorderActionClient>(nh, "RosbagRollingRecord");
    rolling_recorder_ = std::make_shared<Aws::Rosbag::RollingRecorder>(
      bag_rollover_time, max_record_time, write_directory);
  }

  void ClearFilesInPath() {
    if (!boost::filesystem::is_empty(path_)) {
      boost::filesystem::remove_all(path_);
    }
  }

  boost::filesystem::path path_;
  std::shared_ptr<RollingRecorderActionClient> action_client_;
  std::shared_ptr<Aws::Rosbag::RollingRecorder> rolling_recorder_;
};

TEST_F(RollingRecorderNodeFixture, TestGoalReceivedbyActionServer)
{
  ros::AsyncSpinner executor(0);
  executor.start();

  bool message_received = false;
  // Wait 10 seconds for server to start
  ASSERT_TRUE(action_client_->waitForActionServerToStart(ros::Duration(10, 0)));
  auto transition_call_back = [&](RollingRecorderActionClient::GoalHandle goal_handle) {
    if (goal_handle.getCommState() == actionlib::CommState::StateEnum::DONE) {
      EXPECT_EQ(goal_handle.getTerminalState().state_, actionlib::TerminalState::StateEnum::REJECTED);
      message_received = true;
    }
  };
  recorder_msgs::RollingRecorderGoal goal;
  RollingRecorderActionClient::GoalHandle gh = action_client_->sendGoal(goal, transition_call_back);
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
