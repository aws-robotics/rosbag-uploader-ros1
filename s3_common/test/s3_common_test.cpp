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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>
#include <s3_common/s3_facade.h>

using namespace Aws::S3;

class MockS3Client : public S3Client
{
public:
    MockS3Client() : S3Client() {}
    MOCK_METHOD1(PutObject, Model::PutObjectOutcome(const Model::PutObjectRequest &));

};

TEST(S3FacadeTest, TestPutObjectReturns)
{
    auto client = std::make_unique<MockS3Client>();
    S3Facade s3_facade(std::move(client));
    auto result = s3_facade.putObject("file_name", "bucket", "key");
    EXPECT_EQ(S3ErrorCode::SUCCESS, result);
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
