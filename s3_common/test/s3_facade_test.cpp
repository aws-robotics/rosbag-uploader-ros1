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
#include <fstream>
#include <stdlib.h>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>
#include <s3_common/s3_facade.h>
#include <s3_common/utils.h>

using namespace Aws::S3;

using ::testing::Return;
using ::testing::_;

bool FileExists(const std::string& name)
{
  std::ifstream ifile(name);
  return ifile.good();
}

class MockS3Client : public S3Client
{
public:
  MockS3Client() = default;
  MOCK_CONST_METHOD1(PutObject, Model::PutObjectOutcome(const Model::PutObjectRequest &));
};

class S3FacadeTest : public ::testing::Test
{
protected:
  bool enable_encryption = false;
  std::unique_ptr<MockS3Client> client;
  std::string upload_file;
  
  void SetUp() override {
    client = std::make_unique<MockS3Client>();
    char upload_file_path[] = "/tmp/S3FileUploadTestXXXXXX";
    int fd = mkstemp(upload_file_path);
    close(fd);
    upload_file = std::string(upload_file_path);
  }
  void TearDown() override {
    if (FileExists(upload_file)) {
      remove(upload_file.c_str());
    }
  }
};

TEST_F(S3FacadeTest, TestClientConfigConstructor)
{
  Aws::Client::ClientConfiguration config;
  S3Facade s3_facade(enable_encryption, config);
  // No credentials configured, exepect upload to fail
  auto outcome = s3_facade.PutObject(upload_file, "bucket", "key");
  EXPECT_FALSE(outcome.IsSuccess());
}

TEST_F(S3FacadeTest, TestPutObjectSuccess)
{
  // Successful outcome
  Model::PutObjectResult result;
  Model::PutObjectOutcome outcome(result);
  EXPECT_CALL(*client, PutObject(_))
      .WillOnce(Return(outcome));
  auto s3_facade = std::make_shared<S3Facade>(enable_encryption, std::move(client));
  auto facade_outcome = s3_facade->PutObject(upload_file, "bucket", "key");

  EXPECT_TRUE(facade_outcome.IsSuccess());
}

TEST_F(S3FacadeTest, TestPutObjectFileDoesntExist)
{
  // Delete the file so that it doens't exist when trying to read.
  // Note that there is some chance that another file with the generated name
  // is created between deleting this file and trying to open it in PutObject.
  remove(upload_file.c_str());
  auto s3_facade = std::make_shared<S3Facade>(enable_encryption, std::move(client));

  auto outcome = s3_facade->PutObject(upload_file, "bucket", "key");
  EXPECT_FALSE(outcome.IsSuccess());
  EXPECT_EQ(S3Errors::INVALID_PARAMETER_VALUE, outcome.GetError().GetErrorType());   
}

TEST_F(S3FacadeTest, TestPutObjectFailureFromSDK)
{
  Aws::Client::AWSError<S3Errors> error(S3Errors::INTERNAL_FAILURE, false);
  Model::PutObjectOutcome outcome(error);
  EXPECT_CALL(*client, PutObject(_))
      .WillOnce(Return(outcome));
  auto s3_facade = std::make_shared<S3Facade>(enable_encryption, std::move(client));

  auto facade_outcome = s3_facade->PutObject(upload_file, "bucket", "key");
  EXPECT_FALSE(outcome.IsSuccess());
  EXPECT_EQ(S3Errors::INTERNAL_FAILURE, facade_outcome.GetError().GetErrorType());
}
