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
#include <string>

#include <gtest/gtest.h>

#include <s3_common/utils.h>

TEST(S3CommonUtilsTest, TestGetFileNameEmptyStringReturnsEmptyString)
{
    EXPECT_EQ("", GetFileName(""));
}

TEST(S3CommonUtilsTest, TestGetFileNameDirectoryReturnsEmptyString)
{
    EXPECT_EQ("", GetFileName("TestDir/"));
}

TEST(S3CommonUtilsTest, TestGetFileNameNoDirectoryReturnsFileName)
{
    EXPECT_EQ("TestFile", GetFileName("TestFile"));
}

TEST(S3CommonUtilsTest, TestGetFileNameWithDirectoryReturnsFileName)
{
    EXPECT_EQ("TestFile", GetFileName("TestDir/TestFile"));
}

TEST(S3CommonUtilsTest, TestGenerateObjectKeyEmptyPrefixReturnsFileName)
{
    EXPECT_EQ("TestFile", GenerateObjectKey("TestFile", ""));
}

TEST(S3CommonUtilsTest, TestGenerateObjectKeyOnlyUsesFileName)
{
    EXPECT_EQ("prefix/file", GenerateObjectKey("path/to/file", "prefix"));
}

TEST(S3CommonUtilsTest, TestGenerateObjectKeyInsertSlash)
{
    EXPECT_EQ("prefix/file", GenerateObjectKey("file", "prefix"));
}

TEST(S3CommonUtilsTest, TestGenerateObjectKeyDoesntInsertExtraSlash)
{
    EXPECT_EQ("prefix/file", GenerateObjectKey("file", "prefix/"));
}