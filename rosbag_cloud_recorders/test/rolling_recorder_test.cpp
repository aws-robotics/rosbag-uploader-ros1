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

using ::testing::Return;

class MockRollingRecorder : public RollingRecorder
{
public:
  MockRollingRecorder(const ros::Duration & bag_rollover_time, const ros::Duration & max_record_time,
                      std::vector<std::string> topics_to_record, std::string write_directory)
                      : RollingRecorder(bag_rollover_time, max_record_time, topics_to_record, write_directory){};
  MOCK_CONST_METHOD0(IsRollingRecorderActive, bool());

};

class RollingRecorderNodeFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle nh("~");
    std::vector<std::string> topics_to_record;
    topics_to_record.push_back("rosout");
    const ros::Duration max_record_time(5);
    const ros::Duration bag_rollover_time(5);
    action_client_ = std::make_shared<RollingRecorderActionClient>(nh,
      "RosbagRollingRecord");
    rolling_recorder_ = std::make_shared<MockRollingRecorder>(
      bag_rollover_time, max_record_time, topics_to_record, std::string("/tmp/.ros/"));
  }

  std::shared_ptr<RollingRecorderActionClient> action_client_;
  std::shared_ptr<MockRollingRecorder> rolling_recorder_;
};

TEST_F(RollingRecorderNodeFixture, TestGetRollingRecorderHealthStatusApi)
{
  ASSERT_FALSE(rolling_recorder_->IsRollingRecorderActive());
  EXPECT_CALL(*rolling_recorder_, IsRollingRecorderActive())
      .WillOnce(Return(true));
  ASSERT_TRUE(rolling_recorder_->IsRollingRecorderActive());
}

TEST_F(RollingRecorderNodeFixture, TestStartRollingRecorderApi)
{
  EXPECT_CALL(*rolling_recorder_, IsRollingRecorderActive())
      .WillOnce(Return(true));
  EXPECT_EQ(Aws::Rosbag::RecorderErrorCode::RECORDER_IS_RUNNING, rolling_recorder_->StartRollingRecorder());
}

TEST_F(RollingRecorderNodeFixture, TestStopRollingRecorderApi)
{
  EXPECT_EQ(Aws::Rosbag::RecorderErrorCode::RECORDER_NOT_RUNNING, rolling_recorder_->StopRollingRecorder());

  EXPECT_CALL(*rolling_recorder_, IsRollingRecorderActive())
        .WillOnce(Return(true));
  EXPECT_EQ(Aws::Rosbag::RecorderErrorCode::SUCCESS, rolling_recorder_->StopRollingRecorder());
}

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
