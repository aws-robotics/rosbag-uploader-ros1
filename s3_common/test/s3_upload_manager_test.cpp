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
    void feedbackCallback(std::vector<std::string> v)
    {
        num_feedback_calls++;
        EXPECT_EQ(num_feedback_calls, v.size());
    }
};

TEST_F(S3UploadManagerTest, TestUploadFilesSuccess)
{
    EXPECT_CALL(*facade,PutObject(_,_,_))
        .Times(2)
        .WillRepeatedly(Return(S3ErrorCode::SUCCESS));

    S3UploadManager manager(std::move(facade));
    EXPECT_TRUE(manager.IsAvailable());
    auto result = manager.UploadFiles(uploads, "bucket",
        boost::bind(&S3UploadManagerTest::feedbackCallback, this, _1));
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
        boost::bind(&S3UploadManagerTest::feedbackCallback, this, _1));
    EXPECT_EQ(result, S3ErrorCode::FAILED);
    EXPECT_EQ(num_feedback_calls, 1);
    EXPECT_TRUE(manager.IsAvailable());
}

TEST_F(S3UploadManagerTest, TestUploadFilesFailsWhileManagerUploading)
{
    // Pause the execution of the facade to simulate waiting for upload to S3
    std::mutex pause_mutex;
    std::unique_lock<std::mutex> pause_lock(pause_mutex);
    // Used to notify main thread that the upload has started
    std::condition_variable upload_cv;
    std::mutex cv_mutex;
    std::atomic<S3ErrorCode> result1;
    //Keep the upload manager busy
    auto wait_lambda = [&pause_mutex, &upload_cv]()
    {
        upload_cv.notify_one();
        std::unique_lock<std::mutex> lock(pause_mutex);
    };
    EXPECT_CALL(*facade,PutObject(_,_,_))
        .WillOnce(DoAll(
            InvokeWithoutArgs(wait_lambda),
            Return(S3ErrorCode::SUCCESS)))
        .WillOnce(Return(S3ErrorCode::SUCCESS));
    S3UploadManager manager(std::move(facade));
    auto thread = std::thread([&]()
        {
            result1 = manager.UploadFiles(uploads, "bucket",
                boost::bind(&S3UploadManagerTest::feedbackCallback, this, _1));
        }
    );
    // Wait until upload has started so that manager should be busy.
    {
        std::unique_lock<std::mutex> lk(cv_mutex);
        upload_cv.wait(lk);
    }
    EXPECT_FALSE(manager.IsAvailable());

    auto result2 = manager.UploadFiles(uploads, "bucket",
        boost::bind(&S3UploadManagerTest::feedbackCallback, this, _1));
    // The manager is busy and should reject the upload request
    EXPECT_EQ(result2, S3ErrorCode::FAILED);
    pause_lock.unlock();
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
    std::unique_lock<std::mutex> pause_lock(pause_mutex);
    // Used to notify main thread that the upload has started
    std::condition_variable upload_cv;
    std::mutex cv_mutex;
    std::atomic<S3ErrorCode> result;
    //Keep the upload manager busy
    auto wait_lambda = [&pause_mutex, &upload_cv]()
    {
        upload_cv.notify_one();
        std::unique_lock<std::mutex> lock(pause_mutex);
    };
    EXPECT_CALL(*facade,PutObject(_,_,_))
        .WillOnce(DoAll(
            InvokeWithoutArgs(wait_lambda),
            Return(S3ErrorCode::SUCCESS)));
    S3UploadManager manager(std::move(facade));

    // upload hasn't started, nothing to cancel
    EXPECT_FALSE(manager.CancelUpload());
    auto thread = std::thread([&]()
        {
            result = manager.UploadFiles(uploads, "bucket",
                boost::bind(&S3UploadManagerTest::feedbackCallback, this, _1));
        }
    );
    // Wait until upload has started so that manager should be busy.
    {
        std::unique_lock<std::mutex> lk(cv_mutex);
        upload_cv.wait(lk);
    }
    EXPECT_FALSE(manager.IsAvailable());
    bool cancel_result = manager.CancelUpload();
    // Already cancelling, subsequent cancel requests should fail
    EXPECT_FALSE(manager.CancelUpload());
    pause_lock.unlock();
    thread.join();
    EXPECT_TRUE(cancel_result);
    EXPECT_EQ(result, S3ErrorCode::CANCELLED);
    EXPECT_EQ(num_feedback_calls, 1);
    EXPECT_TRUE(manager.IsAvailable());
}
