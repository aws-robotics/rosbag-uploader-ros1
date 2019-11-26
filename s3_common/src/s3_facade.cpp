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
#include <fstream>
#include <string>

#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/s3/model/PutObjectRequest.h>
#include <aws/s3/S3Client.h>

#include <s3_common/s3_common_error_codes.h>
#include <s3_common/s3_facade.h>


#include <aws/s3/model/PutObjectRequest.h>

bool file_exists(const std::string& name)
{
    std::ifstream ifile(name);
    return ifile.good();
}


namespace Aws
{
namespace S3
{

// TODO: Make this configurable
static const int MAX_RETRIES = 3;

S3Facade::S3Facade() 
: s3_client_(std::make_unique<Aws::S3::S3Client>())
{
}

S3Facade::S3Facade(std::unique_ptr<Aws::S3::S3Client> s3_client)
: s3_client_(std::move(s3_client))
{
}


Aws::S3::S3ErrorCode S3Facade::PutObject(
    const std::string & file_path,
    const std::string & bucket,
    const std::string & key)
{
    AWS_LOG_INFO(__func__, "Upload: %s to s3://%s/%s", file_path, bucket, key);
    if (!file_exists(file_path)) {
        AWS_LOG_ERROR(__func__, "Upload failed, file %s couldn't be opened for reading", file_path);
        return Aws::S3::S3ErrorCode::FILE_COULDNT_BE_READ;
    }
    const std::shared_ptr<Aws::IOStream> file_data = 
            std::make_shared<Aws::FStream>(file_path.c_str(), 
                                          std::ios_base::in | std::ios_base::binary);
    Aws::S3::Model::PutObjectRequest putObjectRequest;
    putObjectRequest.SetBucket(bucket.c_str());
    putObjectRequest.SetKey(key.c_str());
    putObjectRequest.SetBody(file_data);

    auto outcome = s3_client_->PutObject(putObjectRequest);

    if (!outcome.IsSuccess()) {
        auto error = outcome.GetError();
        AWS_LOG_ERROR(__func__, "Failed to upload %s to s3://%s/%s with message: %s",
            file_path, bucket, key, error.GetMessage());
        auto error_type = error.GetErrorType();
        if (error_type == Aws::S3::S3Errors::ACCESS_DENIED) {
            return Aws::S3::S3ErrorCode::S3_ACCESS_DENIED;
        } else if (error_type == Aws::S3::S3Errors::NO_SUCH_BUCKET) {
            return Aws::S3::S3ErrorCode::S3_NO_SUCH_BUCKET;
        } else {
            return Aws::S3::S3ErrorCode::FAILED;
        }
                
    } else {
        AWS_LOG_INFO(__func__, "Successfully uploaded %s to s3://%s/%s", file_path, bucket, key);
        return Aws::S3::S3ErrorCode::SUCCESS;
    }
}


}  // namespace S3
}  // namespace Aws
