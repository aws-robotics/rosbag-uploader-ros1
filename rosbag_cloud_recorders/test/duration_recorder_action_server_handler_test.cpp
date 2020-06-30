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

#include <actionlib/client/client_helpers.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>

#include <file_uploader_msgs/UploadFilesAction.h>
#include <recorder_msgs/DurationRecorderAction.h>

#include <rosbag_cloud_recorders/duration_recorder/duration_recorder_action_server_handler.h>
#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>
#include <rosbag_cloud_recorders/duration_recorder/duration_recorder.h>

#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>
#include <boost/filesystem.hpp>
#include <wordexp.h>
#include <stdio.h>
#include <string>

#include <ros/ros.h>

using namespace Aws::Rosbag;

using ::testing::_;
using ::testing::Const;
using ::testing::Eq;
using ::testing::Field;
using ::testing::InSequence;
using ::testing::Property;
using ::testing::Return;
using ::testing::ReturnRef;

MATCHER_P(FeedbackHasStatus, expected_status, "") {
  return expected_status == arg.status.stage;
}

class MockRosbagRecorder : public Utils::RosbagRecorder<Utils::Recorder>
{
public:
  MockRosbagRecorder(): Utils::RosbagRecorder<Utils::Recorder>()
  {
    return_code_ = Utils::RosbagRecorderRunResult::STARTED;
  };
  ~MockRosbagRecorder() {};

  MOCK_CONST_METHOD0(IsActive, bool());
  Utils::RosbagRecorderRunResult Run(
    const Utils::RecorderOptions& recorder_options,
    const std::function<void()>& pre_record,
    const std::function<void(int)>& post_record
  ) {
    options_ = recorder_options;
    if (return_code_ != Utils::RosbagRecorderRunResult::SKIPPED){
      pre_record();
      post_record(rosbag_recorder_exit_code_);
    }
    return return_code_;
  }

  Utils::RecorderOptions getOptions() {
    return options_;
  }

  void SetRosbagRecorderExitCode(int exit_code) {
    rosbag_recorder_exit_code_ = exit_code;
  }
  void SetRosbagRecorderReturnCode(Utils::RosbagRecorderRunResult return_code) {
    return_code_ = return_code;
  }
private:
  Utils::RosbagRecorderRunResult return_code_;
  Utils::RecorderOptions options_;
  int rosbag_recorder_exit_code_;
};

class MockServerGoalHandle
{
  class MockServerGoalHandleImpl
  {
  public:
    MockServerGoalHandleImpl() = default;
    MockServerGoalHandleImpl(const MockServerGoalHandleImpl& copy)
    {
      (void) copy;
    };
    ~MockServerGoalHandleImpl()
    {
    };

    MOCK_METHOD0(setAccepted, void());
    MOCK_METHOD0(setRejected, void());
    MOCK_METHOD0(setCanceled, void());
    MOCK_CONST_METHOD0(getGoal, boost::shared_ptr<recorder_msgs::DurationRecorderGoal>());
    MOCK_METHOD2(setSucceeded, void(const recorder_msgs::DurationRecorderResult&, const std::string &));
    MOCK_METHOD2(setAborted, void(const recorder_msgs::DurationRecorderResult&, const std::string &));
    MOCK_METHOD2(setRejected, void(const recorder_msgs::DurationRecorderResult&, const std::string &));
    MOCK_CONST_METHOD1(publishFeedback, void(recorder_msgs::DurationRecorderFeedback &));
  };

public:
  MockServerGoalHandle() : goal_handle_impl(std::make_shared<MockServerGoalHandleImpl>()) {}
  MockServerGoalHandle(const MockServerGoalHandle & copy) = default;

  MockServerGoalHandleImpl & operator*()
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

  void setRejected(const recorder_msgs::DurationRecorderResult & result, const std::string & msg)
  {
    goal_handle_impl->setRejected(result, msg);
  }

  void publishFeedback(recorder_msgs::DurationRecorderFeedback & feedback)
  {
    goal_handle_impl->publishFeedback(feedback);
  }

private:
  std::shared_ptr<MockServerGoalHandleImpl> goal_handle_impl;
};


class MockS3UploadClient
{
public:
  MOCK_METHOD1(sendGoal,  void(file_uploader_msgs::UploadFilesGoal));
  MOCK_METHOD0(waitForResult, bool());
  MOCK_METHOD1(waitForResult, bool(ros::Duration));
  MOCK_CONST_METHOD0(waitForServer, void());
  MOCK_CONST_METHOD0(isServerConnected, bool());
  MOCK_CONST_METHOD0(getResult, file_uploader_msgs::UploadFilesResultConstPtr());
  MOCK_CONST_METHOD0(getState, actionlib::SimpleClientGoalState());
};

class DurationRecorderActionServerHandlerTests: public ::testing::Test
{
protected:
  MockServerGoalHandle server_goal_handle;
  MockS3UploadClient s3_upload_client;
  std::unique_ptr<MockRosbagRecorder> rosbag_recorder;
  boost::shared_ptr<recorder_msgs::DurationRecorderGoal> goal;
  ros::Duration duration;
  std::vector<std::string> topics_to_record;
  Aws::Rosbag::DurationRecorderOptions duration_recorder_options;
  std::string write_directory;
  boost::filesystem::path path;
public:

  DurationRecorderActionServerHandlerTests():
    goal(new recorder_msgs::DurationRecorderGoal()),
    duration(ros::Duration(5.0)),
    topics_to_record({"/topic1", "/topic2"})
  {
    rosbag_recorder = std::make_unique<MockRosbagRecorder>();
    // Setup for fake bags
    wordexp_t wordexp_result;
    wordexp("~/.ros/dr_handler_test_dir/", &wordexp_result, 0);
    write_directory = *(wordexp_result.we_wordv);
    boost::filesystem::create_directories(write_directory);
    duration_recorder_options.write_directory = write_directory;
    path = boost::filesystem::path(write_directory);
    duration_recorder_options.upload_timeout_s = -1;
    duration_recorder_options.delete_bags_after_upload = false;
  }

  void TearDown() override
  {
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
    std::string bag_name = write_directory + "test_bag_" + std::to_string(bag_count) + ".bag";
    bag.open(bag_name, rosbag::bagmode::Write);
    std_msgs::String str;
    str.data = std::string("foo");
    bag_count++;
    bag.write("topic", time, str);
    bag.close();
    return bag_name;
  }

  void givenDurationRecorderGoal()
  {
    goal->duration = duration;
    goal->topics_to_record = topics_to_record;
    EXPECT_CALL(*server_goal_handle, getGoal()).WillRepeatedly(Return(goal));
  }

  void givenDurationRecorderGoalWithInvalidDuration()
  {
    duration = ros::Duration(-1);
    givenDurationRecorderGoal();
  }

  void givenDurationRecorderGoalWithEmptyTopics()
  {
    topics_to_record = {};
    givenDurationRecorderGoal();
  }

  void givenRecorderActive()
  {
    EXPECT_CALL(*rosbag_recorder, IsActive()).WillRepeatedly(Return(true));
  }

  void givenRecorderBecomesActive()
  {
    rosbag_recorder->SetRosbagRecorderReturnCode(Utils::RosbagRecorderRunResult::SKIPPED);
  }
  
  std::string givenRecorderRanSuccessfully()
  {
    std::string bag_file_name = createRosbagAtTime(ros::Time::now());
    rosbag_recorder->SetRosbagRecorderExitCode(0);
    return bag_file_name;
  }

  void givenRecorderRanUnSuccessfully()
  {
    rosbag_recorder->SetRosbagRecorderExitCode(1);
  }

  void givenRecorderNotActive()
  {
    EXPECT_CALL(*rosbag_recorder, IsActive()).WillRepeatedly(Return(false));
  }

  void givenUploadReturns(actionlib::SimpleClientGoalState state)
  {
    EXPECT_CALL(s3_upload_client, waitForResult()).WillRepeatedly(Return(true));
    EXPECT_CALL(s3_upload_client, getState()).WillRepeatedly(Return(state));
  }

  void givenUploadSucceeds()
  {
    givenUploadReturns(actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED));
    EXPECT_CALL(s3_upload_client, waitForResult(_)).WillRepeatedly(Return(true));
  }

  void givenUploadFails()
  {
    givenUploadReturns(actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED));
  }

  void givenUploadTimesOut()
  {
    // Choose finite wait so that timeout can occur.
    // Note that the test won't wait for this time.
    duration_recorder_options.upload_timeout_s = 1;
    EXPECT_CALL(s3_upload_client, getState()).WillRepeatedly(Return(actionlib::SimpleClientGoalState::StateEnum::ABORTED));
    EXPECT_CALL(s3_upload_client, waitForResult(_)).WillRepeatedly(Return(false));
  }

  void givenDeleteBagAfterUpload(boost::shared_ptr<file_uploader_msgs::UploadFilesResult> result)
  {
    duration_recorder_options.delete_bags_after_upload = true;
    file_uploader_msgs::UploadFilesResultConstPtr result_ptr = result;
    
    EXPECT_CALL(s3_upload_client, getResult()).WillRepeatedly(Return(result_ptr));
  }

  void assertUploadGoalIsSent()
  {
    EXPECT_CALL(s3_upload_client, sendGoal(_));
  }

  void assertGoalIsRejected()
  {
    EXPECT_CALL(*server_goal_handle, setRejected(_, _));
  }

  void assertGoalIsAccepted()
  {
    EXPECT_CALL(*server_goal_handle, setAccepted());
  }

  void assertGoalIsCanceled()
  {
    EXPECT_CALL(*server_goal_handle, setCanceled());
  }

  void assertGoalIsSuccess()
  {
    EXPECT_CALL(*server_goal_handle, setSucceeded(_, _));
  }

  void assertGoalIsAborted()
  {
    EXPECT_CALL(*server_goal_handle, setAborted(_, _));
  }

  void assertPublishFeedback(const std::initializer_list<int> & statuses)
  {
    InSequence s;
    for (const auto status : statuses) {
      EXPECT_CALL(*server_goal_handle, publishFeedback(FeedbackHasStatus(status)));
    }
  }

  void assertRecorderRunWithExpectedOptions()
  {
    auto options = rosbag_recorder->getOptions();
    ASSERT_EQ(options.max_duration, duration);
    if (topics_to_record.empty()) {
      ASSERT_TRUE(rosbag_recorder->getOptions().record_all);
    }
  }

  void assertBagFileCreated(boost::filesystem::path bag_file_path)
  {
    ASSERT_TRUE(boost::filesystem::exists(bag_file_path));
  } 

  void assertBagFileDeleted(boost::filesystem::path bag_file_path)
  {
    ASSERT_FALSE(boost::filesystem::exists(bag_file_path));
  }
};

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderStartSucceeds)
{
  givenRecorderNotActive();
  givenDurationRecorderGoal();
  givenRecorderRanSuccessfully();
  givenUploadSucceeds();
  assertGoalIsAccepted();
  assertUploadGoalIsSent();
  assertPublishFeedback({recorder_msgs::RecorderStatus::RECORDING,
                         recorder_msgs::RecorderStatus::PREPARING_UPLOAD,
                         recorder_msgs::RecorderStatus::UPLOADING,
                         recorder_msgs::RecorderStatus::COMPLETE});
  assertGoalIsSuccess();
  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);

  assertRecorderRunWithExpectedOptions();
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderRecordSucceedsUploadFails)
{
  givenRecorderNotActive();
  givenDurationRecorderGoal();
  givenRecorderRanSuccessfully();
  givenUploadFails();
  assertGoalIsAccepted();
  assertUploadGoalIsSent();
  assertPublishFeedback({recorder_msgs::RecorderStatus::RECORDING,
                         recorder_msgs::RecorderStatus::PREPARING_UPLOAD,
                         recorder_msgs::RecorderStatus::UPLOADING,
                         recorder_msgs::RecorderStatus::COMPLETE});
  assertGoalIsAborted();
  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);

  assertRecorderRunWithExpectedOptions();
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderFailed)
{
  givenRecorderNotActive();
  givenDurationRecorderGoal();
  givenRecorderRanUnSuccessfully();
  assertGoalIsAccepted();
  assertPublishFeedback({recorder_msgs::RecorderStatus::RECORDING});
  assertGoalIsAborted();
  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);

  assertRecorderRunWithExpectedOptions();
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderInvalidDurationGoalFails)
{
  givenRecorderNotActive();
  givenDurationRecorderGoalWithInvalidDuration();
  assertGoalIsRejected();
  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderEmptyTopicsRecordsAll)
{
  givenRecorderNotActive();
  givenDurationRecorderGoalWithEmptyTopics();
  givenRecorderRanSuccessfully();
  givenUploadSucceeds();
  assertGoalIsAccepted();
  assertUploadGoalIsSent();
  assertPublishFeedback({recorder_msgs::RecorderStatus::RECORDING,
                         recorder_msgs::RecorderStatus::PREPARING_UPLOAD,
                         recorder_msgs::RecorderStatus::UPLOADING,
                         recorder_msgs::RecorderStatus::COMPLETE});
  assertGoalIsSuccess();
  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderDeletesFileAfterBagIsUploaded)
{

  boost::shared_ptr<file_uploader_msgs::UploadFilesResult> result(new file_uploader_msgs::UploadFilesResult);
  givenRecorderNotActive();
  givenDurationRecorderGoalWithEmptyTopics();
  std::string bag_file_path = givenRecorderRanSuccessfully();
  result->files_uploaded = std::vector<std::string>{bag_file_path};
  givenUploadSucceeds();
  givenDeleteBagAfterUpload(result);
  assertGoalIsAccepted();
  assertUploadGoalIsSent();
  assertPublishFeedback({recorder_msgs::RecorderStatus::RECORDING,
                         recorder_msgs::RecorderStatus::PREPARING_UPLOAD,
                         recorder_msgs::RecorderStatus::UPLOADING,
                         recorder_msgs::RecorderStatus::COMPLETE,
                         recorder_msgs::RecorderStatus::CLEANUP});
  assertGoalIsSuccess();
  assertBagFileCreated(boost::filesystem::path(bag_file_path));

  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);

  assertBagFileDeleted(boost::filesystem::path(bag_file_path));
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderStartAlreadyActive)
{
  givenRecorderActive();
  assertGoalIsRejected();

  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderStartRecorderBecomesActive)
{
  // Test the case where two goals come through in quick succession. Assert that the second goal is rejected.
  givenRecorderNotActive();
  givenDurationRecorderGoal();
  givenRecorderBecomesActive();
  assertGoalIsRejected();

  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);
}

TEST_F(DurationRecorderActionServerHandlerTests, TestDurationRecorderUploadTimedOut)
{
  givenRecorderNotActive();
  givenDurationRecorderGoal();
  givenRecorderRanSuccessfully();
  givenUploadTimesOut();
  assertGoalIsAccepted();
  assertUploadGoalIsSent();
  assertPublishFeedback({recorder_msgs::RecorderStatus::RECORDING,
                         recorder_msgs::RecorderStatus::PREPARING_UPLOAD,
                         recorder_msgs::RecorderStatus::UPLOADING,
                         recorder_msgs::RecorderStatus::COMPLETE});
  assertGoalIsAborted();
  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::DurationRecorderStart(
    *rosbag_recorder, duration_recorder_options, s3_upload_client, server_goal_handle);
}

TEST_F(DurationRecorderActionServerHandlerTests, TestCancelDurationRecorder)
{
  assertGoalIsCanceled();
  DurationRecorderActionServerHandler<MockServerGoalHandle, MockS3UploadClient>::CancelDurationRecorder(
    server_goal_handle);
}

int main(int argc, char ** argv)
{
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
