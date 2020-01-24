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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <recorder_msgs/DurationRecorderAction.h>

#include<rosbag_cloud_recorders/duration_recorder/duration_recorder_action_server_handler.h>

#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>

#include <rosbag/recorder.h>

#include <ros/ros.h>

#include<rosbag_cloud_recorders/utils/rosbag_recorder.h>

using namespace Aws::Rosbag;

using ::testing::Return;
using ::testing::_;
using ::testing::Field;
using ::testing::Property;
using testing::Eq;

MATCHER_P(FeedbackHasStatus, expected_status, "") {
  return expected_status == arg.status.stage;
}

class MockRosbagRecorder : public Utils::RosbagRecorder
{
public:
  MockRosbagRecorder(rosbag::Recorder& rosbag_recorder): Utils::RosbagRecorder(rosbag_recorder) {};
  ~MockRosbagRecorder() {};
  
  MOCK_CONST_METHOD0(IsActive, bool());
  void Run(const std::function<void()> pre_record, const std::function<void()> post_record) {
    pre_record();
    post_record();
  }
};

class MockGoalHandle 
{
public:
  MockGoalHandle() = default;
  MockGoalHandle(const MockGoalHandle& copy) {
    (void) copy;
  };
  MOCK_METHOD0(setAccepted, void());
  MOCK_METHOD0(setRejected, void());
  MOCK_METHOD0(setCanceled, void());
  MOCK_METHOD2(setSucceeded, void(const recorder_msgs::DurationRecorderResult&, const std::string &));
  
  MOCK_CONST_METHOD1(publishFeedback, void(recorder_msgs::DurationRecorderFeedback &));
};

class DurationRecorderActionServerHandlerTests: public ::testing::Test
{
protected:
  std::shared_ptr<MockGoalHandle> goal_handle;
  std::unique_ptr<MockRosbagRecorder> rosbag_recorder;
public:

  DurationRecorderActionServerHandlerTests():
    goal_handle(std::make_shared<MockGoalHandle>())
  {
    rosbag::RecorderOptions opts;
    rosbag::Recorder recorder(opts);
    rosbag_recorder = std::make_unique<MockRosbagRecorder>(recorder);
  }

  void givenRecorderActive()
  {
    EXPECT_CALL(*rosbag_recorder, IsActive()).Times(1).WillOnce(Return(true));
  }

  void givenRecorderNotActive()
  {
    EXPECT_CALL(*rosbag_recorder, IsActive()).Times(1).WillOnce(Return(false));
  }

  void assertGoalIsRejected()
  {
    EXPECT_CALL(*goal_handle, setRejected());
  }
  
  void assertGoalIsAccepted()
  {
    EXPECT_CALL(*goal_handle, setAccepted());
  }

  void assertGoalIsCanceled()
  {
    EXPECT_CALL(*goal_handle, setCanceled());
  }
  
  void assertGoalIsSuccess()
  {
    EXPECT_CALL(*goal_handle, setSucceeded(_, _));
  }
  
  void assertPublishFeedback()
  {
    recorder_msgs::DurationRecorderFeedback feedback;
    EXPECT_CALL(*goal_handle, publishFeedback(FeedbackHasStatus(recorder_msgs::RecorderStatus::RECORDING)));
  }
};

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderStart)
{
  givenRecorderNotActive();
  assertGoalIsAccepted();
  assertPublishFeedback();
  assertGoalIsSuccess();
  DurationRecorderActionServerHandler<MockGoalHandle>::DurationRecorderStart(*rosbag_recorder, *goal_handle);
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderStartAlreadyActive)
{
  givenRecorderActive();
  assertGoalIsRejected();
  DurationRecorderActionServerHandler<MockGoalHandle>::DurationRecorderStart(*rosbag_recorder, *goal_handle);
}

TEST_F(DurationRecorderActionServerHandlerTests, TestCancelDurationRecorder)
{
  assertGoalIsCanceled();
  DurationRecorderActionServerHandler<MockGoalHandle>::CancelDurationRecorder(*goal_handle);
}

int main(int argc, char ** argv)
{
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
