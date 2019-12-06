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

#include <boost/function.hpp>
#include <string>

#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>

#include <s3_common/s3_common_error_codes.h>
#include <s3_common/s3_facade.h>
#include <s3_common/s3_upload_manager.h>


namespace Aws
{
namespace S3
{

S3UploadManager::S3UploadManager():
    S3UploadManager(std::make_unique<S3Facade>())
{
}

S3UploadManager::S3UploadManager(const Aws::Client::ClientConfiguration &config):
    S3UploadManager(std::make_unique<S3Facade>(config))
{
}

S3UploadManager::S3UploadManager(std::unique_ptr<S3Facade> s3_facade):
    manager_status_(S3UploadManagerState::AVAILABLE),
    s3_facade_(std::move(s3_facade))
{
}

bool S3UploadManager::CancelUpload()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (IsAvailable()) {
        return false;
    }
    manager_status_ = S3UploadManagerState::CANCELLING;
    return true;
}

std::vector<UploadDescription> S3UploadManager::GetCompletedUploads()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return completed_uploads_;
};

S3ErrorCode S3UploadManager::UploadFiles(
        const std::vector<UploadDescription> & upload_descriptions,
        const std::string & bucket,
        boost::function<void (int, int)> feedback_callback)
{
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!IsAvailable()) {
            return S3ErrorCode::UPLOADER_BUSY;
        }
        manager_status_ = S3UploadManagerState::UPLOADING;
        completed_uploads_.clear();
    }

    S3ErrorCode upload_result = S3ErrorCode::SUCCESS;
    int num_files = upload_descriptions.size();
    int num_uploaded = 0;
    for (const auto& upload_description: upload_descriptions) {
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if(manager_status_ == S3UploadManagerState::CANCELLING) {              
                upload_result = S3ErrorCode::CANCELLED;
                break;
            }
        }
        auto file_path = upload_description.file_path;
        auto object_key = upload_description.object_key;
        //bucket comes from config
        AWS_LOGSTREAM_INFO(__func__,"Uploading file " << file_path << " to " << object_key);
        upload_result = s3_facade_->PutObject(file_path, bucket, object_key);
        if (upload_result != S3ErrorCode::SUCCESS) {
            break;
        }
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            completed_uploads_.push_back(upload_description);
        }
        num_uploaded += 1;
        feedback_callback(num_uploaded, num_files-num_uploaded);
    }
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        manager_status_ = S3UploadManagerState::AVAILABLE;
    }
    return upload_result;
}

bool S3UploadManager::IsAvailable() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return manager_status_ == S3UploadManagerState::AVAILABLE;
}

}  // namespace S3
}  // namespace Aws