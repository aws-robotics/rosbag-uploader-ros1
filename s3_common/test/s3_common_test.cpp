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
#include <stdlib.h>
#include <string>
#include <sys/stat.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>
#include <s3_common/s3_facade.h>

using namespace Aws::S3;

using ::testing::Return;
using ::testing::_;


class MockS3Client : public S3Client
{
public:
    MockS3Client() = default;
    MOCK_CONST_METHOD1(PutObject, Model::PutObjectOutcome(const Model::PutObjectRequest &));
};

class S3FacadeTest : public ::testing::Test
{
protected:
    std::unique_ptr<MockS3Client> client;
    std::string upload_file;
    bool uploadFileExists() {
        struct stat buffer;
        return (stat(upload_file.c_str(), &buffer) == 0);
    }
    
    void SetUp() override {
        client = std::make_unique<MockS3Client>();
        char upload_file_path[] = "/tmp/S3FileUploadTestXXXXXX";
        int fd = mkstemp(upload_file_path);
        close(fd);
        upload_file = std::string(upload_file_path);
    }
    void TearDown() override {
        if (uploadFileExists()) {
            remove(upload_file.c_str());
        }
    }
};

TEST_F(S3FacadeTest, TestPutObjectSuccess)
{
    Model::PutObjectResult result;
    Model::PutObjectOutcome outcome(result);
    EXPECT_CALL(*client, PutObject(_))
        .WillOnce(Return(outcome));
    auto s3_facade = std::make_shared<S3Facade>(std::move(client));
    auto facade_result = s3_facade->putObject(upload_file, "bucket", "key");

    EXPECT_EQ(S3ErrorCode::SUCCESS, facade_result);
}

TEST_F(S3FacadeTest, TestPutObjectFileDoesntExist)
{
    // Delete the file so that it doens't exist when trying to read.
    // Note that there is some chance that another file with the generated name
    // is created between deleting this file and trying to open it in putObject.
    remove(upload_file.c_str());
    auto s3_facade = std::make_shared<S3Facade>(std::move(client));

    auto result = s3_facade->putObject(upload_file, "bucket", "key");
    EXPECT_EQ(S3ErrorCode::FILE_COULDNT_BE_READ, result);   
}

TEST_F(S3FacadeTest, TestPutObjectInvalidCredentials)
{
    Aws::Client::AWSError<S3Errors> error(S3Errors::ACCESS_DENIED, false);
    Model::PutObjectOutcome outcome(error);
    EXPECT_CALL(*client, PutObject(_))
        .WillOnce(Return(outcome));
    auto s3_facade = std::make_shared<S3Facade>(std::move(client));

    auto facade_result = s3_facade->putObject(upload_file, "bucket", "key");
    EXPECT_EQ(S3ErrorCode::S3_ACCESS_DENIED, facade_result);
}

TEST_F(S3FacadeTest, TestPutObjectS3BucketDoesntExist)
{
    Aws::Client::AWSError<S3Errors> error(S3Errors::NO_SUCH_BUCKET, false);
    Model::PutObjectOutcome outcome(error);
    EXPECT_CALL(*client, PutObject(_))
        .WillOnce(Return(outcome));
    auto s3_facade = std::make_shared<S3Facade>(std::move(client));

    auto facade_result = s3_facade->putObject(upload_file, "bucket", "key");
    EXPECT_EQ(S3ErrorCode::S3_NO_SUCH_BUCKET, facade_result);
}

int main(int argc, char** argv)
{
    Aws::SDKOptions options;
    Aws::InitAPI(options);
    ::testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    Aws::ShutdownAPI(options);
    return result;
}
