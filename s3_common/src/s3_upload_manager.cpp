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
    manager_status_(S3UploadManagerState::AVAILABLE),
    mutex_(),
    s3_facade_(std::make_unique<S3Facade>())
{
}

S3UploadManager::S3UploadManager(std::unique_ptr<S3Facade> s3_facade):
    manager_status_(S3UploadManagerState::AVAILABLE),
    mutex_(),
    s3_facade_(std::move(s3_facade))
{
}

bool S3UploadManager::cancelUpload()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (manager_status_ != S3UploadManagerState::UPLOADING) {
            return false;
        }
        manager_status_ = S3UploadManagerState::CANCELLING;
    }
    worker_.join();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        manager_status_ = S3UploadManagerState::AVAILABLE;
    }
    return true;
}

S3ErrorCode S3UploadManager::uploadFiles(
        const std::vector<UploadDescription> & upload_descriptions,
        const std::string & bucket,
        boost::function<void (std::vector<std::string>&)> feedback_callback)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (manager_status_ != S3UploadManagerState::AVAILABLE)
        {
            return S3ErrorCode::FAILED;
        } else
        {
            manager_status_ = S3UploadManagerState::UPLOADING;
            upload_result_ = S3ErrorCode::SUCCESS;
        }
    }
    worker_ = std::thread([&]{runUploadFiles(upload_descriptions, bucket, feedback_callback);});
    worker_.join();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        manager_status_ = S3UploadManagerState::AVAILABLE;
    }
    return upload_result_;
}

void S3UploadManager::runUploadFiles(
        const std::vector<UploadDescription> & upload_descriptions,
        const std::string & bucket,
        boost::function<void (std::vector<std::string>&)> feedback_callback)
{
    std::vector<std::string> uploaded_files;
    for (const auto upload_description: upload_descriptions) {
        if(manager_status_ == S3UploadManagerState::CANCELLING) {
            return;
        }
        auto file_path = upload_description.file_path;
        auto object_key = upload_description.object_key;
        //bucket comes from config
        AWS_LOG_INFO(__func__,"Uploading file %s to %s", file_path, object_key);
        auto result = s3_facade_->putObject(file_path, bucket, object_key);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            upload_result_ = result;
        }
        if (result != S3ErrorCode::SUCCESS) {
            return;
        }
        uploaded_files.push_back(file_path);
        feedback_callback(uploaded_files);
    }
}

bool S3UploadManager::isAvailable()
{
    return manager_status_ == S3UploadManagerState::AVAILABLE;
}

}  // namespace S3
}  // namespace Aws