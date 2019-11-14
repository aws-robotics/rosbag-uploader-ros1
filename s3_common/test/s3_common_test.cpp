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
#include <aws/s3/S3Client.h>

#include <s3_common/s3_facade.h>

TEST(S3FacadeTest, TestPutObjectReturns)
{
    auto client = std::make_unique<Aws::S3::S3Client::S3Client>();
    Aws::S3::S3Facade s3_facade(client);
    auto outcome = s3_facade.putObject("file_name");
    EXPECT_TRUE(outcome.IsSuccess())
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
