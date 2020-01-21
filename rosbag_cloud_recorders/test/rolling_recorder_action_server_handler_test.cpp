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
#include <recorder_msgs/RollingRecorderGoal.h>
#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>
#include <ros/ros.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder_action_server_handler.h>

using namespace Aws::Rosbag;

using ::testing::Return;
using ::testing::_;
using ::testing::ContainerEq;
using ::testing::Invoke;

class MockRollingRecorderGoalHandle
{
public:
  MockRollingRecorderGoalHandle() = default;
  MockRollingRecorderGoalHandle(const MockRollingRecorderGoalHandle& copy) {
      (void) copy;
  };
  MOCK_METHOD0(setRejected, void());
  MOCK_METHOD0(setAccepted, void());
  MOCK_METHOD0(setAborted, void());
  MOCK_METHOD0(setSucceeded, void());
  MOCK_METHOD0(setCanceled, void());
  MOCK_METHOD2(setAborted, void(const recorder_msgs::RollingRecorderResult&, const std::string &));
  MOCK_METHOD2(setSucceeded, void(const recorder_msgs::RollingRecorderResult&, const std::string &));

  MOCK_CONST_METHOD0(getGoal, boost::shared_ptr<recorder_msgs::RollingRecorderGoal>());
  MOCK_CONST_METHOD1(publishFeedback, void(recorder_msgs::RollingRecorderFeedback &));
};

class RollingRecorderActionServerHandlerTests: public ::testing::Test
{
protected:
  std::shared_ptr<MockRollingRecorderGoalHandle> goal_handle;
  boost::shared_ptr<recorder_msgs::RollingRecorderGoal> goal;
public:
  RollingRecorderActionServerHandlerTests():
    goal_handle(std::make_shared<MockRollingRecorderGoalHandle>()),
    goal(new recorder_msgs::RollingRecorderGoal()) {}

  void assertGoalIsRejected() {
    EXPECT_CALL(*goal_handle, setRejected());
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

TEST_F(RollingRecorderActionServerHandlerTests, TestRollingRecorderRosbagUpload)
{
  assertGoalIsRejected();

  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle>::RollingRecorderRosbagUpload(*goal_handle);
}

TEST_F(RollingRecorderActionServerHandlerTests, TestCancelRollingRecorderRosbagUpload)
{
  assertGoalIsCanceled();

  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle>::CancelRollingRecorderRosbagUpload(*goal_handle);
}

int main(int argc, char ** argv)
{
    ros::Time::init();
    ::testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
