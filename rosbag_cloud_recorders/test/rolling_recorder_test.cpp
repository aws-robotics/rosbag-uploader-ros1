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
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <unordered_set>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <aws/core/Aws.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <boost/date_time/c_local_time_adjustor.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/console.h>

#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>

#include <recorder_msgs/RollingRecorderAction.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <recorder_msgs/RollingRecorderAction.h>

using namespace Aws::Rosbag;
using RollingRecorderActionClient = actionlib::ActionClient<recorder_msgs::RollingRecorderAction>;

using ::testing::Return;
using ::testing::_;
using ::testing::ContainerEq;
using ::testing::Invoke;

class RollingRecorderTest : public ::testing::Test
{
protected:
  recorder_msgs::RollingRecorderGoal goal;
  ros::AsyncSpinner executor;
  ros::NodeHandle nh;
  RollingRecorderActionClient action_client;
  RollingRecorderActionClient::GoalHandle goal_handle;
  RollingRecorder rolling_recorder_;
  RollingRecorderOptions rolling_recorder_options_;
public:
  RollingRecorderTest():
    executor(0),
    nh("~"),
    action_client(nh, "RosbagRollingRecord")
  {
    char dir_template[] = "/tmp/rolling_recorder_testXXXXXX";
    mkdtemp(dir_template);
    rolling_recorder_options_.bag_rollover_time = ros::Duration(5);
    rolling_recorder_options_.max_record_time = ros::Duration(10);
    rolling_recorder_options_.upload_timeout_s = 3600;
    rolling_recorder_options_.write_directory = std::string(dir_template) + "/";
    executor.start();
  }

  void TearDown() override
  {
    // Delete all files in the write directory to clean up
    boost::filesystem::path path(rolling_recorder_options_.write_directory);
    try {
      boost::filesystem::remove_all(path);
    } catch (std::exception& e) {
      AWS_LOGSTREAM_INFO(__func__, "Caught exception: " << e.what());
    }
  }

  void GivenRollingRecorderInitialized()
  {
    rolling_recorder_.InitializeRollingRecorder(rolling_recorder_options_);
  }

  std::string GetFileNameForTimeStamp(const ros::Time& time)
  {
    std::stringstream file_name;
    boost::posix_time::time_facet *const f =
        new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    boost::posix_time::ptime pt = time.toBoost();
    boost::posix_time::ptime local_pt = boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local(pt);
    file_name.imbue(std::locale(file_name.getloc(),f));
    file_name << local_pt;
    return file_name.str();
  }

  std::string CreateRosBagFileStartingAtTime(const ros::Time& time, std::string suffix)
  {
    std::string file_name = rolling_recorder_options_.write_directory + GetFileNameForTimeStamp(time) + suffix;
    std::fstream file;
    file.open(file_name, std::ios::out);
    file.close();
    return file_name;
  }

  std::vector<std::string> GivenOldRosBags(int num_bags)
  {
    std::vector<std::string> rosbags;
    for (int i = 0; i < num_bags; ++i) {
      std::string suffix = "_" + std::to_string(i) + ".bag";
      rosbags.emplace_back(CreateRosBagFileStartingAtTime(ros::Time::now() - rolling_recorder_options_.max_record_time - ros::Duration(1000), suffix));
    }
    return rosbags;
  }

  std::vector<std::string> GivenRecentRosBags(int num_bags)
  {
    std::vector<std::string> rosbags;
    for (int i = 0; i < num_bags; ++i) {
      std::string suffix = "_" + std::to_string(i) + ".bag";
      rosbags.emplace_back(CreateRosBagFileStartingAtTime(ros::Time::now(), suffix));
    }
    return rosbags;
  }

  std::vector<std::string> GivenInvalidFileNames(int num_bags)
  {
    std::vector<std::string> rosbags;
    for (int i = 0; i < num_bags; ++i) {
      std::string suffix = "_" + std::to_string(i) + ".bag";
      std::string file_name = rolling_recorder_options_.write_directory + "myInvalidBagWithoutADate" + suffix;
      std::fstream file;
      file.open(file_name, std::ios::out);
      file.close();
      rosbags.emplace_back(file_name);
    }
    return rosbags;
  }

  bool AllFilesInFilesToDeleteHaveCountComparison(const std::vector<std::string>& bags, boost::function<bool(int)> comparator)
  {
    auto files_to_delete = rolling_recorder_.GetRosBagsToDelete();
    std::unordered_set<std::string> files_set(files_to_delete.begin(), files_to_delete.end());
    for (const auto& bag: bags) {
      if (!comparator(files_set.count(bag))) {
        return false;
      }
    }
    return true;
  }

  bool FilesToDeleteContainsAllOf(const std::vector<std::string>& bags)
  {
    return AllFilesInFilesToDeleteHaveCountComparison(bags, [](int count) -> bool {return count != 0;});
  }

  bool FilesToDeleteContainsNoneOf(const std::vector<std::string>& bags)
  {
    return AllFilesInFilesToDeleteHaveCountComparison(bags, [](int count) -> bool {return count == 0;});
  }
};

TEST_F(RollingRecorderTest, TestConstructorWithValidParamInput)
{
  ros::Duration max_record_time(5);
  ros::Duration bag_rollover_time(5);

  rolling_recorder_options_.max_record_time = max_record_time;
  rolling_recorder_options_.bag_rollover_time = bag_rollover_time;
  {
    Aws::Rosbag::RollingRecorder rolling_recorder;
  }
}

TEST_F(RollingRecorderTest, TestInvalidParamInput)
{
  // Case: bag_rollover_time > max_record_time
  {
    ros::Duration max_record_time(5);
    ros::Duration bag_rollover_time(6);
    rolling_recorder_options_.max_record_time = max_record_time;
    rolling_recorder_options_.bag_rollover_time = bag_rollover_time;

    EXPECT_FALSE(rolling_recorder_.ValidInputParam(rolling_recorder_options_));
  }

  // Case: bag_rollover_time < 0
  {
    ros::Duration max_record_time(-5);
    ros::Duration bag_rollover_time(6);
    rolling_recorder_options_.max_record_time = max_record_time;
    rolling_recorder_options_.bag_rollover_time = bag_rollover_time;

    EXPECT_FALSE(rolling_recorder_.ValidInputParam(rolling_recorder_options_));
  }

  // Case: bag_rollover_time < 0
  {
    ros::Duration max_record_time(5);
    ros::Duration bag_rollover_time(-6);
    rolling_recorder_options_.max_record_time = max_record_time;
    rolling_recorder_options_.bag_rollover_time = bag_rollover_time;

    EXPECT_FALSE(rolling_recorder_.ValidInputParam(rolling_recorder_options_));
  }
}

TEST_F(RollingRecorderTest, TestActionReceived)
{
  GivenRollingRecorderInitialized();

  bool message_received = false;
  // Wait 10 seconds for server to start
  ASSERT_TRUE(action_client.waitForActionServerToStart(ros::Duration(10, 0)));
  auto transition_call_back = [&message_received](RollingRecorderActionClient::GoalHandle goal_handle){
    if (goal_handle.getCommState().state_ == actionlib::CommState::StateEnum::DONE) {
      EXPECT_EQ(goal_handle.getTerminalState().state_, actionlib::TerminalState::StateEnum::SUCCEEDED);
      message_received = true;
    }
  };
  recorder_msgs::RollingRecorderGoal goal;
  auto gh = action_client.sendGoal(goal, transition_call_back);
  
  ros::Duration(1,0).sleep();
  ASSERT_TRUE(message_received);
}

TEST_F(RollingRecorderTest, TestGetRosBagsToDeleteDeletesOldBags)
{
  GivenRollingRecorderInitialized();
  auto old_file_names = GivenOldRosBags(3);
  auto recent_file_names = GivenRecentRosBags(3);
  auto invalid_file_names = GivenInvalidFileNames(3);
  EXPECT_TRUE(FilesToDeleteContainsAllOf(old_file_names));
  EXPECT_TRUE(FilesToDeleteContainsNoneOf(recent_file_names));
  EXPECT_TRUE(FilesToDeleteContainsNoneOf(invalid_file_names));
}

TEST_F(RollingRecorderTest, TestUpdateStatusEffectOnGetRosBagsToDelete)
{
  GivenRollingRecorderInitialized();
  file_uploader_msgs::UploadFilesGoal upload_goal;
  upload_goal.files = GivenOldRosBags(3);
  RollingRecorderStatus status;
  status.SetUploadGoal(upload_goal);
  // Update the status to include the files and expect them to be protected from deletion.
  rolling_recorder_.UpdateStatus(status);
  EXPECT_TRUE(FilesToDeleteContainsNoneOf(upload_goal.files));
  // Reset the status and expect the files to now be included in the result.
  rolling_recorder_.UpdateStatus(RollingRecorderStatus());
  EXPECT_TRUE(FilesToDeleteContainsAllOf(upload_goal.files));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_rolling_recorder_node");
  Aws::Utils::Logging::InitializeAWSLogging(
        Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("test_rolling_recorder_node"));
  auto result = RUN_ALL_TESTS();
  Aws::Utils::Logging::ShutdownAWSLogging();
  ros::shutdown();
  return result;
}
