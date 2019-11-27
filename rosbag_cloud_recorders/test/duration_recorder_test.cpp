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
#include <fstream>
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <recorder_msgs/DurationRecorderAction.h>
#include <recorder_common_error_codes.h>
#include <duration_recorder/duration_recorder.h>
#include <utils/rosbag_file_manager.h>

using namespace Aws::Rosbag;
using namespace rosbag;


using DurationRecorderActionClient = actionlib::ActionClient<recorder_msgs::DurationRecorderAction>;

class DurationRecorderNodeFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle nh("~");
    action_client_ = std::make_shared<DurationRecorderActionClient>(nh, "RosbagDurationRecord");
    duration_recorder_ = std::make_shared<Aws::Rosbag::DurationRecorder>();
    rosbag_file_manager_ = std::make_shared<Aws::Rosbag::Utils::RosbagFileManager>();
  }

  std::shared_ptr<DurationRecorderActionClient> action_client_;
  std::shared_ptr<Aws::Rosbag::DurationRecorder> duration_recorder_;
  std::shared_ptr<Aws::Rosbag::Utils::RosbagFileManager> rosbag_file_manager_;
};

TEST_F(DurationRecorderNodeFixture, TestActionReceivedbyActionServer)
{
  ros::AsyncSpinner executor(0);
  executor.start();
  bool message_received = false;
  // Wait 10 seconds for server to start
  ASSERT_TRUE(action_client_->waitForActionServerToStart(ros::Duration(10, 0)));
  auto transition_call_back = [&message_received](DurationRecorderActionClient::GoalHandle goal_handle){
      EXPECT_EQ(goal_handle.getTerminalState().state_, actionlib::TerminalState::StateEnum::REJECTED);
      message_received = true;
  };
  recorder_msgs::DurationRecorderGoal goal;
  auto gh = action_client_->sendGoal(goal, transition_call_back);
  ros::Duration(1,0).sleep();
  ASSERT_TRUE(message_received);
  executor.stop();
}

TEST_F(DurationRecorderNodeFixture, TestRosbagRemovalSuccessfulCase)
{
  std::ofstream file("./TestRosbagRemovalSuccessfulCase.bag");
  file.close();
  EXPECT_EQ(rosbag_file_manager_->DeleteRosbag("./TestRosbagRemovalSuccessfulCase.bag"), Aws::Rosbag::RecorderErrorCode::SUCCESS);
}

TEST_F(DurationRecorderNodeFixture, TestRosbagRemovalFailureCases)
{
  EXPECT_EQ(rosbag_file_manager_->DeleteRosbag("/I/Am/Nowhere/To/Be/Found.bag"), Aws::Rosbag::RecorderErrorCode::ROSBAG_FILE_NOT_FOUND);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_duration_recorder_node");
  auto result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
