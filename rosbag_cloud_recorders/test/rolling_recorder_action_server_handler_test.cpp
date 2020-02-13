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
  class MockRollingRecorderGoalHandleImpl
  {
  public:
    MockRollingRecorderGoalHandleImpl() = default;
    MockRollingRecorderGoalHandleImpl(const MockRollingRecorderGoalHandleImpl& copy)
    {
      (void) copy;
    };
    ~MockRollingRecorderGoalHandleImpl()
    {
    };

    MOCK_METHOD0(setAccepted, void());
    MOCK_METHOD0(setRejected, void());
    MOCK_METHOD0(setCanceled, void());
    MOCK_CONST_METHOD0(getGoal, boost::shared_ptr<recorder_msgs::RollingRecorderGoal>());
    MOCK_METHOD2(setSucceeded, void(const recorder_msgs::RollingRecorderResult&, const std::string &));
    MOCK_METHOD2(setAborted, void(const recorder_msgs::RollingRecorderResult&, const std::string &));
    MOCK_METHOD2(setRejected, void(const recorder_msgs::RollingRecorderResult&, const std::string &));
    MOCK_CONST_METHOD1(publishFeedback, void(recorder_msgs::RollingRecorderFeedback &));
  };

public:
  MockRollingRecorderGoalHandle() : goal_handle_impl(std::make_shared<MockRollingRecorderGoalHandleImpl>()) {}
  MockRollingRecorderGoalHandle(const MockRollingRecorderGoalHandle & copy) = default;

  MockRollingRecorderGoalHandleImpl & operator*()
  {
    return *goal_handle_impl;
  }

  void setAccepted()
  {
    goal_handle_impl->setAccepted();
  }

  void setRejected()
  {
    goal_handle_impl->setRejected();
  }

  void setCanceled()
  {
    goal_handle_impl->setCanceled();
  }

  boost::shared_ptr<recorder_msgs::RollingRecorderGoal> getGoal()
  {
    return goal_handle_impl->getGoal();
  }

  void setSucceeded(const recorder_msgs::RollingRecorderResult & result, const std::string & msg)
  {
    goal_handle_impl->setSucceeded(result, msg);
  }

  void setAborted(const recorder_msgs::RollingRecorderResult & result, const std::string & msg)
  {
    goal_handle_impl->setAborted(result, msg);
  }

  void setRejected(const recorder_msgs::RollingRecorderResult & result, const std::string & msg)
  {
    goal_handle_impl->setRejected(result, msg);
  }

  void publishFeedback(recorder_msgs::RollingRecorderFeedback & feedback)
  {
    goal_handle_impl->publishFeedback(feedback);
  }

private:
  std::shared_ptr<MockRollingRecorderGoalHandleImpl> goal_handle_impl;
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
  MockRollingRecorderGoalHandle goal_handle;
  //std::shared_ptr<recorder_msgs::RollingRecorderGoal> goal;
  boost::shared_ptr<recorder_msgs::RollingRecorderGoal> goal;
  std::atomic<bool> action_server_busy;
  MockS3UploadClient s3_upload_client;
  boost::filesystem::path path;
  RollingRecorderOptions rolling_recorder_options;

public:
  RollingRecorderActionServerHandlerTests():
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
    // Delete all files in the write directory to clean up
    try {
      boost::filesystem::remove_all(path);
    } catch (std::exception& e) {
      AWS_LOGSTREAM_INFO(__func__, "Caught exception: " << e.what());
    }
  }

  std::string createRosbagAtTime(ros::Time time)
  {
    static int bag_count = 0;
    rosbag::Bag bag;
    std::string bag_name = rolling_recorder_options.write_directory + "test_bag_" + std::to_string(bag_count);
    bag.open(bag_name, rosbag::bagmode::Write);
    std_msgs::String str;
    str.data = std::string("foo");
    bag_count++;
    bag.write("topic", time, str);
    bag.close();
    return bag_name;
  }

  void givenRollingRecorderGoal()
  {
    EXPECT_CALL(*goal_handle, getGoal()).WillRepeatedly(Return(goal));
  }

  void givenActionServerBusy()
  {
    action_server_busy = true;
  }

  void givenActionServerAvailable()
  {
    action_server_busy = false;
  }

  void givenRecorderIsRunning()
  {
    createRosbagAtTime(ros::Time::now()-rolling_recorder_options.bag_rollover_time);
  }

  void givenUploadReturns(actionlib::SimpleClientGoalState state)
  {
    EXPECT_CALL(s3_upload_client, waitForResult(_)).WillRepeatedly(Return(true));
    EXPECT_CALL(s3_upload_client, getState()).WillRepeatedly(Return(state));
  }

  void givenUploadSucceeds()
  {
    givenUploadReturns(actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED));
  }

  void givenUploadFails()
  {
    givenUploadReturns(actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED));
  }

  void givenUploadTimesOut()
  {
    // Choose finite wait so that timeout can occur.
    // Note that the test won't wait for this time.
    rolling_recorder_options.upload_timeout_s = 1;
    EXPECT_CALL(s3_upload_client, getState()).WillRepeatedly(Return(actionlib::SimpleClientGoalState::StateEnum::ABORTED));
    EXPECT_CALL(s3_upload_client, waitForResult(_)).WillRepeatedly(Return(false));
  }

  void assertUploadGoalIsSent()
  {
    EXPECT_CALL(s3_upload_client, sendGoal(_));
  }

  void assertGoalIsRejected()
  {
    EXPECT_CALL(*goal_handle, setRejected(_, _));
  }

  void assertGoalIsAccepted()
  {
    EXPECT_CALL(*goal_handle, setAccepted());
  }

  void assertGoalIsSuccess()
  {
    EXPECT_CALL(*goal_handle, setSucceeded(_, _));
  }

  void assertGoalIsCanceled()
  {
    EXPECT_CALL(*goal_handle, setCanceled());
  }

  void assertGoalIsAborted()
  {
    EXPECT_CALL(*goal_handle, setAborted(_, _));
  }

  void assertPublishFeedback()
  {
    EXPECT_CALL(*goal_handle, publishFeedback(_));
  }

};

TEST_F(RollingRecorderActionServerHandlerTests, TestRollingRecorderActionSucceeds)
{
  givenActionServerAvailable();
  givenUploadSucceeds();
  givenRecorderIsRunning();
  givenRollingRecorderGoal();
  assertGoalIsAccepted();
  assertPublishFeedback();
  assertUploadGoalIsSent();
  assertGoalIsSuccess();
  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle, MockS3UploadClient>::RollingRecorderRosbagUpload(
    goal_handle,
    rolling_recorder_options,
    s3_upload_client,
    action_server_busy);
}

TEST_F(RollingRecorderActionServerHandlerTests, TestRollingRecorderUploadFails)
{
  givenActionServerAvailable();
  givenUploadFails();
  givenRecorderIsRunning();
  givenRollingRecorderGoal();
  assertGoalIsAccepted();
  assertPublishFeedback();
  assertUploadGoalIsSent();
  assertGoalIsAborted();
  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle, MockS3UploadClient>::RollingRecorderRosbagUpload(
    goal_handle,
    rolling_recorder_options,
    s3_upload_client,
    action_server_busy);
}

TEST_F(RollingRecorderActionServerHandlerTests, TestRollingRecorderUploadTimesOut)
{
  givenActionServerAvailable();
  givenUploadTimesOut();
  givenRecorderIsRunning();
  givenRollingRecorderGoal();
  assertGoalIsAccepted();
  assertPublishFeedback();
  assertUploadGoalIsSent();
  assertGoalIsAborted();
  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle, MockS3UploadClient>::RollingRecorderRosbagUpload(
    goal_handle,
    rolling_recorder_options,
    s3_upload_client,
    action_server_busy);
}

TEST_F(RollingRecorderActionServerHandlerTests, TestRollingRecorderActionServerBusy)
{
  // Test when action server is processing a goal
  givenActionServerBusy();
  assertGoalIsRejected();

  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle, MockS3UploadClient>::RollingRecorderRosbagUpload(
    goal_handle,
    rolling_recorder_options,
    s3_upload_client,
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
