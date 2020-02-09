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

#include <rosbag_cloud_recorders/duration_recorder/duration_recorder_action_server_handler.h>
#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>
#include <rosbag_cloud_recorders/duration_recorder/duration_recorder.h>

#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>

#include <ros/ros.h>

using namespace Aws::Rosbag;

using ::testing::Return;
using ::testing::_;
using ::testing::Field;
using ::testing::Property;
using testing::Eq;

MATCHER_P(FeedbackHasStatus, expected_status, "") {
  return expected_status == arg.status.stage;
}

class MockRosbagRecorder : public Utils::RosbagRecorder<Utils::Recorder>
{
public:
  MockRosbagRecorder(): Utils::RosbagRecorder<Utils::Recorder>() {};
  ~MockRosbagRecorder() {};
  
  MOCK_CONST_METHOD0(IsActive, bool());
  Utils::RosbagRecorderRunResult Run(
    const Utils::RecorderOptions& recorder_options,
    const std::function<void()>& pre_record,
    const std::function<void(int)>& post_record
  ) {
    options_ = recorder_options;
    pre_record();
    post_record(rosbag_recorder_exit_code_);
    return Utils::RosbagRecorderRunResult::STARTED;
  }
  
  Utils::RecorderOptions getOptions() {
    return options_;
  }
  
  void SetRosbagRecorderExitCode(int exit_code) {
    rosbag_recorder_exit_code_ = exit_code;
  }
private:
  Utils::RecorderOptions options_;
  int rosbag_recorder_exit_code_;
};

class MockGoalHandle
{
  class MockGoalHandleImpl
  {
  public:
    MockGoalHandleImpl() = default;
    MockGoalHandleImpl(const MockGoalHandleImpl& copy)
    {
      (void) copy;
    };
    ~MockGoalHandleImpl()
    {
    };

    MOCK_METHOD0(setAccepted, void());
    MOCK_METHOD0(setRejected, void());
    MOCK_METHOD0(setCanceled, void());
    MOCK_CONST_METHOD0(getGoal, boost::shared_ptr<recorder_msgs::DurationRecorderGoal>());
    MOCK_METHOD2(setSucceeded, void(const recorder_msgs::DurationRecorderResult&, const std::string &));
    MOCK_METHOD2(setAborted, void(const recorder_msgs::DurationRecorderResult&, const std::string &));

    MOCK_CONST_METHOD1(publishFeedback, void(recorder_msgs::DurationRecorderFeedback &));
  };

public:
  MockGoalHandle() : goal_handle_impl(std::make_shared<MockGoalHandleImpl>()) {}
  MockGoalHandle(const MockGoalHandle & copy) = default;

  MockGoalHandleImpl & operator*()
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

  boost::shared_ptr<recorder_msgs::DurationRecorderGoal> getGoal()
  {
    return goal_handle_impl->getGoal();
  }

  void setSucceeded(const recorder_msgs::DurationRecorderResult & result, const std::string & msg)
  {
    goal_handle_impl->setSucceeded(result, msg);
  }

  void setAborted(const recorder_msgs::DurationRecorderResult & result, const std::string & msg)
  {
    goal_handle_impl->setAborted(result, msg);
  }

  void publishFeedback(recorder_msgs::DurationRecorderFeedback & feedback)
  {
    goal_handle_impl->publishFeedback(feedback);
  }

private:
  std::shared_ptr<MockGoalHandleImpl> goal_handle_impl;
};

class DurationRecorderActionServerHandlerTests: public ::testing::Test
{
protected:
  MockGoalHandle goal_handle;
  std::unique_ptr<MockRosbagRecorder> rosbag_recorder;
  boost::shared_ptr<recorder_msgs::DurationRecorderGoal> goal;
  ros::Duration duration;
  std::vector<std::string> topics_to_record;
  Aws::Rosbag::DurationRecorderOptions duration_recorder_options;
public:

  DurationRecorderActionServerHandlerTests():
    goal(new recorder_msgs::DurationRecorderGoal()),
    duration(ros::Duration(5.0)),
    topics_to_record({"/topic1", "/topic2"})
  {
    rosbag_recorder = std::make_unique<MockRosbagRecorder>();
  }
  
  void givenDurationRecorderGoal()
  {
    goal->duration = duration;
    goal->topics_to_record = topics_to_record;
    EXPECT_CALL(*goal_handle, getGoal()).Times(1).WillOnce(Return(goal));
  }

  void givenRecorderActive()
  {
    EXPECT_CALL(*rosbag_recorder, IsActive()).Times(1).WillRepeatedly(Return(true));
  }
  
  void givenRecorderRanSuccessfully()
  {
    rosbag_recorder->SetRosbagRecorderExitCode(0);
  }
  
  void givenRecorderRanUnSuccessfully()
  {
    rosbag_recorder->SetRosbagRecorderExitCode(1);
  }

  void givenRecorderNotActive()
  {
    EXPECT_CALL(*rosbag_recorder, IsActive()).Times(1).WillRepeatedly(Return(false));
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
  
  void assertGoalIsAborted()
  {
    EXPECT_CALL(*goal_handle, setAborted(_, _));
  }
  
  void assertPublishFeedback()
  {
    recorder_msgs::DurationRecorderFeedback feedback;
    EXPECT_CALL(*goal_handle, publishFeedback(FeedbackHasStatus(recorder_msgs::RecorderStatus::RECORDING)));
  }
  
  void assertRecorderRunWithExpectedOptions()
  {
    auto options = rosbag_recorder->getOptions();
    ASSERT_EQ(options.max_duration, duration);
  }

};
/*
TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderStart)
{
  givenRecorderNotActive();
  givenDurationRecorderGoal();
  givenRecorderRanSuccessfully();
  assertGoalIsAccepted();
  assertPublishFeedback();
  assertGoalIsSuccess();
  DurationRecorderActionServerHandler<MockGoalHandle>::DurationRecorderStart(*rosbag_recorder, duration_recorder_options, goal_handle);
  
  assertRecorderRunWithExpectedOptions();
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderFailed)
{
  givenRecorderNotActive();
  givenDurationRecorderGoal();
  givenRecorderRanUnSuccessfully();
  assertGoalIsAccepted();
  assertPublishFeedback();
  assertGoalIsAborted();
  DurationRecorderActionServerHandler<MockGoalHandle>::DurationRecorderStart(*rosbag_recorder, duration_recorder_options, goal_handle);
  
  assertRecorderRunWithExpectedOptions();
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderStartAlreadyActive)
{
  givenRecorderActive();
  assertGoalIsRejected();

  DurationRecorderActionServerHandler<MockGoalHandle>::DurationRecorderStart(*rosbag_recorder, duration_recorder_options, goal_handle);
}

TEST_F(DurationRecorderActionServerHandlerTests, TestCancelDurationRecorder)
{
  assertGoalIsCanceled();
  DurationRecorderActionServerHandler<MockGoalHandle>::CancelDurationRecorder(goal_handle);
}
*/
int main(int argc, char ** argv)
{
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
