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
  ros::Duration bag_rollover_time;
  ros::Duration max_record_time;
  std::string write_directory;
  std::shared_ptr<RollingRecorder> rolling_recorder_;

public:
  RollingRecorderTest():
    executor(0),
    nh("~"),
    action_client(nh, "RosbagRollingRecord"),
    bag_rollover_time(5),
    max_record_time(10)
  {
    executor.start();
    char dir_template[] = "/tmp/rolling_recorder_testXXXXXX";
    mkdtemp(dir_template);
    write_directory = std::string(dir_template) + "/";
  }

  void TearDown() override
  {
    // Delete all files in the write directory to clean up
    boost::filesystem::path path(write_directory);
    boost::filesystem::remove_all(path);
  }

  void GivenRollingRecorder()
  {
    rolling_recorder_ = std::make_shared<RollingRecorder>(bag_rollover_time, max_record_time, write_directory);
  }
  
  std::string GetFileNameForTimeStamp(const ros::Time& time)
  {
    std::stringstream file_name;
    boost::posix_time::time_facet *const f=
        new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    boost::posix_time::ptime pt = time.toBoost();
    boost::posix_time::ptime local_pt = boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local(pt);
    file_name.imbue(std::locale(file_name.getloc(),f));
    file_name << local_pt;
    return file_name.str();
  }

  std::string CreateRosBagFileStartingAtTime(const ros::Time& time, std::string suffix)
  {
    std::string file_name = write_directory + GetFileNameForTimeStamp(time) + suffix;
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
      rosbags.emplace_back(CreateRosBagFileStartingAtTime(ros::Time::now() - max_record_time - ros::Duration(1000), suffix));
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
      std::string file_name = write_directory + "myInvalidBagWithoutADate" + suffix;
      std::fstream file;
      file.open(file_name, std::ios::out);
      file.close();
      rosbags.emplace_back(file_name);
    }
    return rosbags;
  }

  bool AllFilesInFilesToDeleteHaveCountComparison(const std::vector<std::string>& bags, boost::function<bool(int)> comparator)
  {
    auto files_to_delete = rolling_recorder_->GetRosBagsToDelete();
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

TEST_F(RollingRecorderTest, TestConstructor)
{
  ros::Duration max_record_time(5);
  ros::Duration bag_rollover_time(5);

  {
    Aws::Rosbag::RollingRecorder rolling_recorder(bag_rollover_time, max_record_time, write_directory);
  }
}

TEST_F(RollingRecorderTest, TestGetRosBagsToDeleteDeletesOldBags)
{
  GivenRollingRecorder();
  auto old_file_names = GivenOldRosBags(3);
  auto recent_file_names = GivenRecentRosBags(3);
  auto invalid_file_names = GivenInvalidFileNames(3);
  EXPECT_TRUE(FilesToDeleteContainsAllOf(old_file_names));
  EXPECT_TRUE(FilesToDeleteContainsNoneOf(recent_file_names));
  EXPECT_TRUE(FilesToDeleteContainsNoneOf(invalid_file_names));
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
