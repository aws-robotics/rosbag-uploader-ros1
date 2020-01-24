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
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <std_msgs/String.h>
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
  std::string write_directory;
  ros::Duration bag_rollover_time;
  boost::filesystem::path path;

public:
  RollingRecorderActionServerHandlerTests():
    goal_handle(std::make_shared<MockRollingRecorderGoalHandle>()),
    goal(new recorder_msgs::RollingRecorderGoal()),
    write_directory(std::string(std::getenv("HOME")) + "/.ros/"),
    bag_rollover_time(ros::Duration(30, 0)),
    path(write_directory) {}

  void SetUp() override
  {
    // Delete all files in the write directory for start testing
    boost::filesystem::remove_all(path);
    boost::filesystem::create_directory(path);
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

  void createRosbagInWriteDirectory(std::vector<std::string> bag_files) {
    for(std::string bag_file : bag_files)  {

      std::ofstream new_bag_file(write_directory + bag_file);
    }
  }

  void checkGetRosbagsToUploadGetsValidFileExtension(ros::Time time_of_goal_received) {
    std::vector<std::string>  bag_files_in_write_directory = Aws::Rosbag::RollingRecorderActionServerHandler<MockRollingRecorderGoalHandle>::GetRosbagsToUpload(write_directory, bag_rollover_time, time_of_goal_received);
    for (std::string bag_file_in_write_directory : bag_files_in_write_directory) {
      EXPECT_EQ(boost::filesystem::extension(bag_file_in_write_directory), ".bag");
    }
  }
  void constructTestRosbagFile(std::string ros_bag_file_name, ros::Time time_of_message_created) {
    rosbag::Bag new_ros_bag;
    new_ros_bag.open(ros_bag_file_name, rosbag::bagmode::Write);


    std_msgs::String test_message;
    test_message.data = std::string("Running unit tests rolling recorder.");

    new_ros_bag.write("rolling_recorder_unit_test", time_of_message_created, test_message);
    new_ros_bag.close();
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

TEST_F(RollingRecorderActionServerHandlerTests, TestGetRosbagToUpload)
{
  ros::Time time_of_function_called(ros::Time::now());

  // All valid bag file extension
  {
    std::vector<std::string> bag_files_to_create{"test1.bag", "test2.bag"};
    createRosbagInWriteDirectory(std::move(bag_files_to_create));

    checkGetRosbagsToUploadGetsValidFileExtension(time_of_function_called);
  }

  // All invalid bag file extension
  {
    std::vector<std::string> bag_files_to_create{"test1.bag.test", "test1.bag.random", "test1.bag.how.about.this.Bag", "text_file.txt"};
    createRosbagInWriteDirectory(std::move(bag_files_to_create));

    checkGetRosbagsToUploadGetsValidFileExtension(time_of_function_called);
  }

  // Test inactive file case
  {
    std::string test_ros_bag_file_name("test_active.bag");
    ros::Time time_of_message_created(time_of_function_called - ros::Duration(24*60*60));  // 1 day before the test gets ran
    std::vector<std::string> bag_files_to_create{test_ros_bag_file_name + ".active"};
    createRosbagInWriteDirectory(std::move(bag_files_to_create));
    std::thread process_active_bag_file_thread([&] () { checkGetRosbagsToUploadGetsValidFileExtension(time_of_function_called); } );
    ros::Duration(3, 0).sleep();
    ASSERT_TRUE(boost::filesystem::remove(write_directory + test_ros_bag_file_name + ".active"));
    constructTestRosbagFile(write_directory + test_ros_bag_file_name, time_of_message_created);
    process_active_bag_file_thread.join();
  }
}

int main(int argc, char ** argv)
{
    ros::Time::init();
    ::testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
