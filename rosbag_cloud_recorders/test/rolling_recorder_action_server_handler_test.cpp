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

#include <fstream>
#include <thread>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <boost/ref.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <rosbag/bag.h>
#include <recorder_msgs/RollingRecorderGoal.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder_action_server_handler.h>
#include <aws_common/fs_utils/wordexp_ros.h>
#include <wordexp.h>

using namespace Aws::Rosbag;

using ::testing::_;
using ::testing::Return;
using ::testing::UnorderedElementsAre;

class MockRollingRecorderGoalHandle
{
public:
  MockRollingRecorderGoalHandle() = default;
  MockRollingRecorderGoalHandle(const MockRollingRecorderGoalHandle& copy) {
      (void) copy;
  };

  MOCK_METHOD0(setAccepted, void());
  MOCK_METHOD0(setCanceled, void());

  MOCK_METHOD2(setRejected, void(const recorder_msgs::RollingRecorderResult&, const std::string &));
  MOCK_METHOD2(setAborted, void(const recorder_msgs::RollingRecorderResult&, const std::string &));
  MOCK_METHOD2(setSucceeded, void(const recorder_msgs::RollingRecorderResult&, const std::string &));

  MOCK_CONST_METHOD0(getGoal, std::shared_ptr<recorder_msgs::RollingRecorderGoal>());
  MOCK_CONST_METHOD1(publishFeedback, void(recorder_msgs::RollingRecorderFeedback &));
};

class MockS3UploadClient
{
public:
  MOCK_METHOD1(sendGoal,  void(file_uploader_msgs::UploadFilesGoal));
  MOCK_METHOD0(waitForResult, bool());
  MOCK_METHOD1(waitForResult, bool(ros::Duration));
  MOCK_CONST_METHOD0(waitForServer, void());
  MOCK_CONST_METHOD0(isServerConnected, bool());
  MOCK_CONST_METHOD0(getState, actionlib::SimpleClientGoalState());
};

class RollingRecorderActionServerHandlerTests: public ::testing::Test
{
protected:
  std::shared_ptr<MockRollingRecorderGoalHandle> goal_handle;
  std::shared_ptr<recorder_msgs::RollingRecorderGoal> goal;
  std::atomic<bool> action_server_busy;
  MockS3UploadClient rosbag_uploader_action_client;
  boost::filesystem::path path;
  RollingRecorderOptions rolling_recorder_options;

public:
  RollingRecorderActionServerHandlerTests():
    goal_handle(std::make_shared<MockRollingRecorderGoalHandle>()),
    goal(new recorder_msgs::RollingRecorderGoal()),
    action_server_busy(false)
  {
    wordexp_t wordexp_result;
    wordexp("~/.ros/rr_handler_test_dir/", &wordexp_result, 0);
    rolling_recorder_options.write_directory = *(wordexp_result.we_wordv);
    rolling_recorder_options.upload_timeout_s = 3600;
    rolling_recorder_options.bag_rollover_time = ros::Duration(10);
    rolling_recorder_options.max_record_time = ros::Duration(100);
    path = boost::filesystem::path(rolling_recorder_options.write_directory);
  }

  void SetUp() override
  {
    // Delete all files in the write directory for start testing
    boost::filesystem::remove_all(path);
    boost::filesystem::create_directories(path);
  }

  void TearDown() override {
    // Delete all files in the write directory for cleaning up
    boost::filesystem::remove_all(path);
  }

  void assertGoalIsRejected() {
    EXPECT_CALL(*goal_handle, setRejected(_, _));
  }

  void assertGoalIsAccepted() {
    EXPECT_CALL(*goal_handle, setAccepted());
  }

  void assertGoalIsSuccess() {
    EXPECT_CALL(*goal_handle, setSucceeded(_, _));
  }

  void assertGoalIsCanceled() {
    EXPECT_CALL(*goal_handle, setCanceled());
  }

  void assertGoalIsAborted() {
    EXPECT_CALL(*goal_handle, setAborted(_, _));
  }
};

TEST_F(RollingRecorderActionServerHandlerTests, TestRollingRecorderRosbagUpload_ActionServerIsBusy)
{
  // Test when action server is processing a goal
  action_server_busy = true;

  assertGoalIsRejected();

  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle, MockS3UploadClient>::RollingRecorderRosbagUpload(
    *goal_handle,
    rolling_recorder_options,
    rosbag_uploader_action_client,
    action_server_busy);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_rosbag_rolling_recorder");
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
