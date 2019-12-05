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
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <string>
#include <boost/bind.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <s3_common/s3_common_error_codes.h>
#include <s3_common/s3_facade.h>
#include <s3_common/s3_upload_manager.h>

using namespace Aws::S3;
using ::testing::DoAll;
using ::testing::InvokeWithoutArgs;
using ::testing::IgnoreResult;
using ::testing::Return;
using ::testing::_;

class MockS3Facade : public S3Facade
{
public:
    MockS3Facade() : S3Facade() {}
    MOCK_METHOD3(PutObject, S3ErrorCode(const std::string &, const std::string &, const std::string &));
};


// This function simulates a busy process that will block until mutex is unlocked
void WaitUntilUnlocked(
    std::mutex& mutex,
    std::condition_variable& cv)
{
    // Notify that the function is entered and blocking
    cv.notify_all();
    // Block until the mutex has been unlocked
    std::unique_lock<std::mutex> lock(mutex);
}

class S3UploadManagerTest : public ::testing::Test
{
protected:
    std::unique_ptr<MockS3Facade> facade;
    std::vector<UploadDescription> uploads;
    int num_feedback_calls;
    void SetUp() override
    {
        num_feedback_calls = 0;
        facade = std::make_unique<MockS3Facade>();
        uploads = 
            {
                {"file1", "location1"},
                {"file2", "location2"}
            };
    }
public:
    void FeedbackCallback(int num_uploaded, int num_remaining)
    {
        num_feedback_calls++;
        EXPECT_EQ(num_uploaded, num_feedback_calls);
        EXPECT_EQ(num_remaining, uploads.size()-num_uploaded);
    }

    std::thread UploadFilesUntilUnlocked(S3UploadManager& manager, S3ErrorCode& result) {
        return std::thread([&]()
            {
                result = manager.UploadFiles(uploads, "bucket",
                    [this](int num_uploaded, int num_remaining) {this->FeedbackCallback(num_uploaded, num_remaining);});
            }
        );
    } 
};


TEST_F(S3UploadManagerTest, TestUploadFilesSuccess)
{
    EXPECT_CALL(*facade,PutObject(_,_,_))
        .Times(2)
        .WillRepeatedly(Return(S3ErrorCode::SUCCESS));

    S3UploadManager manager(std::move(facade));
    EXPECT_TRUE(manager.IsAvailable());
    EXPECT_TRUE(manager.GetCompletedUploads().empty());
    auto result = manager.UploadFiles(uploads, "bucket",
        [this](int num_uploaded, int num_remaining) {this->FeedbackCallback(num_uploaded, num_remaining);});
    EXPECT_EQ(result, S3ErrorCode::SUCCESS);
    EXPECT_EQ(num_feedback_calls, uploads.size());
    EXPECT_TRUE(manager.IsAvailable());
}

TEST_F(S3UploadManagerTest, TestUploadFilesFailsPutObjectFails)
{
    EXPECT_CALL(*facade,PutObject(_,_,_))
        .WillOnce(Return(S3ErrorCode::SUCCESS))
        .WillOnce(Return(S3ErrorCode::FAILED));
    S3UploadManager manager(std::move(facade));
    EXPECT_TRUE(manager.IsAvailable());
    auto result = manager.UploadFiles(uploads, "bucket",
        [this](int num_uploaded, int num_remaining) {this->FeedbackCallback(num_uploaded, num_remaining);});
    EXPECT_EQ(result, S3ErrorCode::FAILED);
    EXPECT_EQ(num_feedback_calls, 1);
    EXPECT_EQ(manager.GetCompletedUploads().size(), 1);
    EXPECT_TRUE(manager.IsAvailable());
}

TEST_F(S3UploadManagerTest, TestUploadFilesFailsWhileManagerUploading)
{
    // Pause the execution of the facade to simulate waiting for upload to S3
    std::mutex pause_mutex;
    // Used to notify main thread that the upload has started
    std::condition_variable upload_cv;
    S3ErrorCode result1;


    EXPECT_CALL(*facade, PutObject(_,_,_))
        .WillOnce(DoAll(
            InvokeWithoutArgs([&upload_cv, &pause_mutex](){WaitUntilUnlocked(pause_mutex, upload_cv);}),
            Return(S3ErrorCode::SUCCESS)))
        .WillOnce(Return(S3ErrorCode::SUCCESS));
    
    S3UploadManager manager(std::move(facade));

    // Pause execution of file upload
    pause_mutex.lock();

    auto thread = UploadFilesUntilUnlocked(manager, result1);

    // Wait until upload has started so that manager should be busy.
    {
        std::mutex cv_mutex;
        std::unique_lock<std::mutex> lk(cv_mutex);
        upload_cv.wait(lk);
    }

    EXPECT_FALSE(manager.IsAvailable());

    auto result2 = manager.UploadFiles(uploads, "bucket",
        [this](int num_uploaded, int num_remaining) {this->FeedbackCallback(num_uploaded, num_remaining);});
    // The manager is busy and should reject the upload request
    EXPECT_EQ(result2, S3ErrorCode::FAILED);
    // No files have been uploaded
    EXPECT_TRUE(manager.GetCompletedUploads().empty());

    // Finish execution of file upload
    pause_mutex.unlock();
    thread.join();

    // The first request should continue uninterrupted
    EXPECT_EQ(result1, S3ErrorCode::SUCCESS);
    EXPECT_EQ(num_feedback_calls, uploads.size());
    EXPECT_TRUE(manager.IsAvailable());
}

TEST_F(S3UploadManagerTest, TestCancelUpload)
{
    // Pause the execution of the facade to simulate waiting for upload to S3
    std::mutex pause_mutex;
    // Used to notify main thread that the upload has started
    std::condition_variable upload_cv;
    S3ErrorCode result;

    EXPECT_CALL(*facade,PutObject(_,_,_))
        .WillOnce(DoAll(
            InvokeWithoutArgs([&upload_cv, &pause_mutex](){WaitUntilUnlocked(pause_mutex, upload_cv);}),
            Return(S3ErrorCode::SUCCESS)));

    S3UploadManager manager(std::move(facade));

    // upload hasn't started, nothing to cancel
    EXPECT_FALSE(manager.CancelUpload());

    // Pause execution of file upload
    pause_mutex.lock();

    auto thread = UploadFilesUntilUnlocked(manager, result);
    
    // Wait until upload has started so that manager should be busy.
    {
        std::mutex cv_mutex;
        std::unique_lock<std::mutex> lk(cv_mutex);
        upload_cv.wait(lk);
    }

    EXPECT_FALSE(manager.IsAvailable());
    // First call to cancel should succeed
    EXPECT_TRUE(manager.CancelUpload());
    // Subsequent cancel request should still succeed
    EXPECT_TRUE(manager.CancelUpload());

    // Finish execution of file upload
    pause_mutex.unlock();
    thread.join();

    EXPECT_EQ(result, S3ErrorCode::CANCELLED);
    EXPECT_EQ(num_feedback_calls, 1);
    EXPECT_EQ(manager.GetCompletedUploads().size(), 1);
    EXPECT_TRUE(manager.IsAvailable());
}
