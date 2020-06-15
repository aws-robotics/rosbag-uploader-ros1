/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <gmock/gmock.h>

#include <s3_common/s3_upload_manager.h>

class MockS3UploadManager : public Aws::S3::S3UploadManager
{
public:
  MockS3UploadManager() : Aws::S3::S3UploadManager(false) {}

  MOCK_METHOD3(UploadFiles, Aws::S3::Model::PutObjectOutcome(
    const std::vector<Aws::S3::UploadDescription> &,
    const std::string &,
    const boost::function<void(const std::vector<Aws::S3::UploadDescription> &)> &));

  MOCK_METHOD0(CancelUpload, void());

  MOCK_CONST_METHOD0(IsAvailable,bool());
};
