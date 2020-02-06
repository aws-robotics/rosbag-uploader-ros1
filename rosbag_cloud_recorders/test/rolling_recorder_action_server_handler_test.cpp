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
#include <recorder_msgs/RollingRecorderGoal.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder_action_server_handler.h>
#include <aws_common/fs_utils/wordexp_ros.h>
#include <wordexp.h>

using namespace Aws::Rosbag;

using ::testing::_;
using ::testing::UnorderedElementsAre;

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

  MOCK_CONST_METHOD0(getGoal, std::shared_ptr<recorder_msgs::RollingRecorderGoal>());
  MOCK_CONST_METHOD1(publishFeedback, void(recorder_msgs::RollingRecorderFeedback &));
};

class RollingRecorderActionServerHandlerTests: public ::testing::Test
{
protected:
  std::shared_ptr<MockRollingRecorderGoalHandle> goal_handle;
  std::shared_ptr<recorder_msgs::RollingRecorderGoal> goal;
  std::string write_directory;
  ros::Duration bag_rollover_time;
  boost::filesystem::path path;

public:
  RollingRecorderActionServerHandlerTests():
    goal_handle(std::make_shared<MockRollingRecorderGoalHandle>()),
    goal(new recorder_msgs::RollingRecorderGoal()),
    bag_rollover_time(ros::Duration(10, 0))
  {
    wordexp_t wordexp_result;
    wordexp("~/.ros/rr_handler_test_dir/", &wordexp_result, 0);
    write_directory = *(wordexp_result.we_wordv);
    path = boost::filesystem::path(write_directory);
  }

  void SetUp() override
  {
    // Delete all files in the write directory for start testing
    boost::filesystem::remove_all(path);
    boost::filesystem::create_directories(path);
  }

  void TearDown() override
  {
    // Delete all files in the write directory for cleaning up
    boost::filesystem::remove_all(path);
  }

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

  void createRosbagInWriteDirectory(std::vector<std::string> bag_filenames) {
    for (std::string bag_filename : bag_filenames)  {
      auto bag_file = rosbag::Bag(write_directory + bag_filename, rosbag::bagmode::Write);
      std_msgs::String str_msg;
      str_msg.data = std::string("foo");
      bag_file.write("/topic", ros::Time::now(), str_msg);
      bag_file.close();
    }
  }

  std::vector<std::string> checkGetRosbagsToUploadFileExtensions(ros::Time time_of_goal_received) {
    std::vector<std::string> bag_files_in_write_directory = Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle>::GetRosbagsToUpload(write_directory, bag_rollover_time, time_of_goal_received);
    for (std::string bag_file_in_write_directory : bag_files_in_write_directory) {
      EXPECT_EQ(boost::filesystem::extension(bag_file_in_write_directory), ".bag");
    }
    return bag_files_in_write_directory;
  }

};

TEST_F(RollingRecorderActionServerHandlerTests, TestRollingRecorderRosbagUpload)
{
  assertGoalIsRejected();

  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle>::RollingRecorderRosbagUpload(*goal_handle, write_directory, bag_rollover_time);
}

TEST_F(RollingRecorderActionServerHandlerTests, TestCancelRollingRecorderRosbagUpload)
{
  assertGoalIsCanceled();

  Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle>::CancelRollingRecorderRosbagUpload(*goal_handle);
}

TEST_F(RollingRecorderActionServerHandlerTests, TestGetRosbagToUploadMixedFiles)
{
  createRosbagInWriteDirectory(std::vector<std::string>({"test1.bag", "nonbagfile", "test2.bag"}));

  auto goal_received_time = ros::Time::now();
  // We want to make absolutely sure that bags created after setting goal_received_time will have a later start time. Short sleep will do.
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));

  createRosbagInWriteDirectory(std::vector<std::string>({"test_late1.bag", "nonbagfile2", "test_late2.bag"}));

  EXPECT_THAT(checkGetRosbagsToUploadFileExtensions(goal_received_time),
              UnorderedElementsAre(write_directory + "test1.bag", write_directory + "test2.bag"));
}

TEST_F(RollingRecorderActionServerHandlerTests, TestGetRosbagToUploadNoEligibleFiles)
{
  ros::Time time_of_function_called(ros::Time::now());
  std::vector<std::string> bag_files_to_create{"test1.bag.test", "test1.bag.random", "test1.bag.how.about.this.Bag", "text_file.txt"};
  createRosbagInWriteDirectory(std::move(bag_files_to_create));
  EXPECT_EQ(checkGetRosbagsToUploadFileExtensions(time_of_function_called),
            std::vector<std::string>({}));
}

int main(int argc, char ** argv)
{
    ros::Time::init();
    ::testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
