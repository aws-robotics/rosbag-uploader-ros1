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
using ::testing::Return;
using ::testing::_;

class MockS3Facade : public S3Facade
{
public:
    MockS3Facade() : S3Facade() {}
    MOCK_METHOD3(putObject, S3ErrorCode(const std::string &, const std::string &, const std::string &));
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
    EXPECT_CALL(*facade,putObject(_,_,_))
        .Times(2)
        .WillRepeatedly(Return(S3ErrorCode::SUCCESS));

    S3UploadManager manager(std::move(facade));
    EXPECT_TRUE(manager.isAvailable());
    auto result = manager.uploadFiles(uploads, "bucket",
        boost::bind(&S3UploadManagerTest::feedbackCallback, this, _1));
    EXPECT_EQ(result, S3ErrorCode::SUCCESS);
    EXPECT_EQ(num_feedback_calls, uploads.size());
    EXPECT_TRUE(manager.isAvailable());
}

TEST_F(S3UploadManagerTest, TestUploadFilesFailsPutObjectFails)
{
    EXPECT_CALL(*facade,putObject(_,_,_))
        .WillOnce(Return(S3ErrorCode::SUCCESS))
        .WillOnce(Return(S3ErrorCode::FAILED));
    S3UploadManager manager(std::move(facade));
    EXPECT_TRUE(manager.isAvailable());
    auto result = manager.uploadFiles(uploads, "bucket",
        boost::bind(&S3UploadManagerTest::feedbackCallback, this, _1));
    EXPECT_EQ(result, S3ErrorCode::FAILED);
    EXPECT_EQ(num_feedback_calls, 1);
    EXPECT_TRUE(manager.isAvailable());
}


TEST(S3UploadManagerTest, TestUploadFilesFailsManagerUploading)
{
}
/*
TEST(S3UploadManagerTest, TestCancelUploadSucceeds)
{
}
TEST(S3UploadManagerTest, TestCancelUploadFailsNotUploading)
{
}*/
