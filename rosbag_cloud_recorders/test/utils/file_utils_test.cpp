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
#include <chrono>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <sys/stat.h>
#include <thread>

#include <cstring>
#include <cerrno>
#include <string>
#include <unistd.h>
#include <wordexp.h>

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rosbag_cloud_recorders/utils/file_utils.h>
#include <aws_common/fs_utils/wordexp_ros.h>


using namespace Aws::Rosbag::Utils;
using ::testing::_;
using ::testing::UnorderedElementsAre;

class ExpandAndCreateDirTests: public ::testing::Test
{
public:
  ExpandAndCreateDirTests()
  {
    // Delete any existing test directory for start testing
    boost::filesystem::remove_all(test_dir);

    old_home = getenv("HOME");
  }

  ~ExpandAndCreateDirTests()
  {
    // Delete any existing test directory for cleaning up
    boost::filesystem::remove_all(test_dir);

    if (old_home == nullptr) {
      unsetenv("HOME");
    } else {
      setenv("HOME", old_home, true);
    }
  }

protected:
  static constexpr char test_dir[] = "./temp_file_utils_test_dir";
  static constexpr char test_dir_str[] = "~/temp_file_utils_test_dir";

private:
  const char * old_home;
};

constexpr char ExpandAndCreateDirTests::test_dir[];
constexpr char ExpandAndCreateDirTests::test_dir_str[];

class GetRosbagToUploadTests: public ::testing::Test
{
public:
  GetRosbagToUploadTests()
  {
    wordexp_t wordexp_result;
    wordexp("~/.ros/file_utils_test_dir/", &wordexp_result, 0);
    write_directory = *(wordexp_result.we_wordv);
    path = boost::filesystem::path(write_directory);
  }

  void SetUp() override
  {
    ros::Time::init();
    // Delete all files in the write directory for start testing
    boost::filesystem::remove_all(path);
    boost::filesystem::create_directories(path);
  }

  void TearDown() override
  {
    // Delete all files in the write directory for cleaning up
    boost::filesystem::remove_all(path);
  }

  void createRosbagInWriteDirectory(std::vector<std::string> bag_filenames) {
    for (std::string bag_filename : bag_filenames)  {
      rosbag::Bag bag_file;
      bag_file.open(write_directory + bag_filename, rosbag::bagmode::Write);
      std_msgs::String str_msg;
      str_msg.data = std::string("foo");
      bag_file.write("/topic", ros::Time::now(), str_msg);
      bag_file.close();
    }
  }

  std::vector<std::string> checkGetRosbagsToUploadFileExtensions(ros::Time time_of_goal_received) {
    std::vector<std::string> bag_files_in_write_directory = Aws::Rosbag::Utils::GetRosbagsToUpload(write_directory, 
        [time_of_goal_received](rosbag::View& rosbag) -> bool {return time_of_goal_received >= rosbag.getBeginTime();}
    );
    for (std::string bag_file_in_write_directory : bag_files_in_write_directory) {
      EXPECT_EQ(boost::filesystem::extension(bag_file_in_write_directory), ".bag");
    }
    return bag_files_in_write_directory;
  }

protected:
  std::string write_directory;
  boost::filesystem::path path;
};

TEST_F(ExpandAndCreateDirTests, TestForNonexistingDirectory)
{
  std::string expanded_dir;
  setenv("HOME", ".", true);
  bool success = ExpandAndCreateDir(test_dir_str, expanded_dir);
  ASSERT_EQ(test_dir, expanded_dir);
  ASSERT_TRUE(success);
  ASSERT_TRUE(boost::filesystem::exists(expanded_dir));
}

TEST_F(ExpandAndCreateDirTests, TestForExistingDirectory)
{
  // place an existing directory where it should be
  boost::filesystem::create_directories(test_dir);

  // test
  std::string expanded_dir;
  setenv("HOME", ".", true);
  bool success = ExpandAndCreateDir(test_dir_str, expanded_dir);
  ASSERT_EQ(test_dir, expanded_dir);
  ASSERT_TRUE(success);
  ASSERT_TRUE(boost::filesystem::exists(expanded_dir));
}

TEST_F(ExpandAndCreateDirTests, TestForNonwriteableDirectory)
{
  using namespace boost::filesystem;

  // place an existing directory where it should be
  create_directories(test_dir);
  permissions(test_dir, owner_read | group_read | others_read);

  // test
  std::string expanded_dir;
  setenv("HOME", ".", true);
  bool success = ExpandAndCreateDir(test_dir_str, expanded_dir);
  ASSERT_EQ(test_dir, expanded_dir);
  ASSERT_FALSE(success);  // this test will fail if run as root
}

TEST_F(ExpandAndCreateDirTests, TestForImpossibleDirectory)
{
  // create a file where the directory should be
  std::ofstream temp_file(test_dir);  
  temp_file.close();

  // test
  std::string expanded_dir;
  setenv("HOME", ".", true);
  bool success = ExpandAndCreateDir(test_dir_str, expanded_dir);
  ASSERT_EQ(test_dir, expanded_dir);
  ASSERT_FALSE(success);
}

TEST(DeleteFileTest, TestFileRemovalSucceeds)
{
    std::string path = "./TestRosbagRemovalSuccessfulCase.bag";
    std::ofstream file(path);
    file.close();
    EXPECT_EQ(DeleteFile(path), Aws::Rosbag::RecorderErrorCode::SUCCESS);
}

TEST(DeleteFileTest, TestFileRemovalFailsFileDoesntExist)
{
  EXPECT_EQ(DeleteFile("/I/Am/Nowhere/To/Be/Found.bag"), Aws::Rosbag::RecorderErrorCode::FILE_NOT_FOUND);
}

TEST(DeleteFileTest, TestRemoveDirectoryFails)
{
    char dir_template[] = "/tmp/DeleteFileTestXXXXXX";
    mkdtemp(dir_template);
    EXPECT_EQ(Aws::Rosbag::RecorderErrorCode::FILE_REMOVAL_FAILED, DeleteFile(dir_template));
    rmdir(dir_template);
}

TEST(GetRosBagStartTimeTest, SucceedsOnProperlyFormattedInputs)
{
    // Test all possible combinations of properly formatted time stamp with/without directory,
    // bag number, file extension
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("2020-01-14-17-13-14"));
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("2020-01-14-17-13-14_1"));
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("2020-01-14-17-13-14_1.bag"));
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("dir/dir/2020-01-14-17-13-14_1.bag"));
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("2020-01-14-17-13-14.bag"));
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("dir/dir/2020-01-14-17-13-14.bag"));
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("dir/dir/2020-01-14-17-13-14_1"));
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("dir/2020-01-14-17-13-14"));
    EXPECT_EQ(ros::Time(1579021994), GetRosBagStartTime("dir/_2020-01-14-17-13-14_2.bag"));
}

TEST(GetRosBagStartTimeTest, ReturnsZeroOnInvalidInput)
{
    // Input is invalid date is not proper format
    EXPECT_TRUE(GetRosBagStartTime("dir/dir/2020-01-14-17-13-14/1.bag").isZero());
    // Input is invalid because the date is correctly formatted but is too far in the future
    EXPECT_TRUE(GetRosBagStartTime("3020-01-14-17-13-14_1.bag").isZero());
}

TEST_F(GetRosbagToUploadTests, TestGetRosbagToUploadMixedFiles)
{
  createRosbagInWriteDirectory(std::vector<std::string>({"test1.bag", "nonbagfile", "test2.bag"}));

  auto goal_received_time = ros::Time::now();
  // We want to make absolutely sure that bags created after setting goal_received_time will have a later start time. Short sleep will do.
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));

  createRosbagInWriteDirectory(std::vector<std::string>({"test_late1.bag", "nonbagfile2", "test_late2.bag"}));

  EXPECT_THAT(checkGetRosbagsToUploadFileExtensions(goal_received_time),
              UnorderedElementsAre(write_directory + "test1.bag", write_directory + "test2.bag"));
}

TEST_F(GetRosbagToUploadTests, TestGetRosbagToUploadNoEligibleFiles)
{
  ros::Time time_of_function_called(ros::Time::now());
  std::vector<std::string> bag_files_to_create{"test1.bag.test", "test1.bag.random", "test1.bag.how.about.this.Bag", "text_file.txt"};
  createRosbagInWriteDirectory(std::move(bag_files_to_create));
  EXPECT_EQ(checkGetRosbagsToUploadFileExtensions(time_of_function_called),
            std::vector<std::string>({}));
}
