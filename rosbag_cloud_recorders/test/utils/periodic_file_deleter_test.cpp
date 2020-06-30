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
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rosbag_cloud_recorders/utils/file_utils.h>
#include <rosbag_cloud_recorders/utils/periodic_file_deleter.h>

using namespace Aws::Rosbag::Utils;

class PeriodicFileDeleterTest : public ::testing::Test
{
public:
    std::vector<std::string> GetFiles() {
        get_files_count += 1;
        return delete_files;
    };
protected:
    std::vector<std::string> files;
    std::vector<std::string> delete_files;
    std::unique_ptr<PeriodicFileDeleter> deleter;
    int get_files_count;

    std::string CreateTempFile() {
        char file_path[] = "/tmp/PeriodicFileDeleterTestXXXXXX";
        int fd = mkstemp(file_path);
        close(fd);
        auto file = std::string(file_path);
        // Track files so that they can be cleaned up after the test
        files.push_back(file);
        return file;
    }

    void CreateDeleteFiles(int num_files) {
        for (int i = 0; i < num_files; ++i) {
            delete_files.push_back(CreateTempFile());
        }
    }

    bool FileExists(std::string file_path) {
        std::ifstream file(file_path);
        return file.good();
    }

    void SetUp() override
    {
        get_files_count = 0;
        deleter = std::make_unique<PeriodicFileDeleter>(boost::bind(&PeriodicFileDeleterTest::GetFiles, this));
    }

    void TearDown() override
    {
        // In case a test fails, delete all files
        for (const auto& file: files) {
            Aws::Rosbag::Utils::DeleteFile(file.c_str());
        }
    }
};

TEST_F(PeriodicFileDeleterTest, TestIsActive)
{
    ASSERT_FALSE(deleter->IsActive());
    deleter->Start();
    ASSERT_TRUE(deleter->IsActive());
    // Shouldn't do anything
    deleter->Start();
    ASSERT_TRUE(deleter->IsActive());
    deleter->Stop();
    ASSERT_FALSE(deleter->IsActive());
}

TEST_F(PeriodicFileDeleterTest, TestDeletesFilesSucceeds)
{
    int num_files = 3;
    CreateDeleteFiles(num_files);

    // Sanity check that files were created
    ASSERT_EQ(num_files, delete_files.size());
    for (const auto& file: delete_files) {
        ASSERT_TRUE(FileExists(file));
    }

    deleter->Start();

    // Deleter won't call GetFiles again until it's completed the first batch
    while (get_files_count < 2) {
        sleep(1);
    }
    for (const auto& file: delete_files) {
        EXPECT_FALSE(FileExists(file));
    }
}

TEST_F(PeriodicFileDeleterTest, TestIgnoresFailedFileDeletion)
{
    int num_files = 3;
    CreateDeleteFiles(num_files);
    // Sanity check that files were created
    ASSERT_EQ(num_files, delete_files.size());
    for (const auto& file: delete_files) {
        ASSERT_TRUE(FileExists(file));
    }

    // Delete a file before the deleter can
    DeleteFile(delete_files.at(1));

    deleter->Start();

    // Deleter won't call GetFiles again until it's completed the first batch
    while (get_files_count < 2) {
        sleep(1.0);
    }
    // All files should still be succesfully deleted
    for (const auto& file: delete_files) {
        EXPECT_FALSE(FileExists(file));
    }
}