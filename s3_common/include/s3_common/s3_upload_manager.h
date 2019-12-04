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
#pragma once

#include <memory>
#include <string>
#include <boost/function.hpp>

#include <s3_common/s3_common_error_codes.h>
#include <s3_common/s3_facade.h>

namespace Aws
{
namespace S3
{

enum S3UploadManagerState
{
    // The upload manager isn't uploading any files
    AVAILABLE = 0,
    // The upload manager is uploading files
    UPLOADING,
    // The upload manager is canceling a request to upload files
    CANCELLING
};

struct UploadDescription
{
    std::string file_path;
    std::string object_key;
};

class S3UploadManager
{
public:
    S3UploadManager();
    S3UploadManager(std::unique_ptr<S3Facade> s3_facade);
    virtual ~S3UploadManager() = default;

    virtual bool CancelUpload();
    virtual S3ErrorCode UploadFiles(
        const std::vector<UploadDescription> & upload_descriptions,
        const std::string & bucket,
        boost::function<void (std::vector<std::string>&)> feedback_callback);
    virtual bool IsAvailable() const;
private:
    void RunUploadFiles(
        const std::vector<UploadDescription> & upload_descriptions,
        const std::string & bucket,
        boost::function<void (std::vector<std::string>&)> feedback_callback);

    S3UploadManagerState manager_status_;
    mutable std::mutex mutex_;
    std::unique_ptr<S3Facade> s3_facade_;
    S3ErrorCode upload_result_;
    std::thread worker_;
};



}  // namespace S3
}  // namespace Aws