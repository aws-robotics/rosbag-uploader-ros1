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

#include<rosbag_cloud_recorders/duration_recorder/duration_recorder_action_server_handler.h>

#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>

#include<rosbag_cloud_recorders/utils/rosbag_recorder.h>

using namespace Aws::Rosbag;

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
};

class DurationRecorderActionServerHandlerTests: public ::testing::Test
{
protected:
  std::shared_ptr<MockGoalHandle> goal_handle;
  std::unique_ptr<Utils::RosbagRecorder> rosbag_recorder;
public:
  DurationRecorderActionServerHandlerTests():
    goal_handle(std::make_shared<MockGoalHandle>())
  {
  }

  void assertGoalIsRejected() {
    EXPECT_CALL(*goal_handle, setRejected());
  }

  void assertGoalIsCanceled() {
    EXPECT_CALL(*goal_handle, setCanceled());
  }
};

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderStart)
{
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
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
