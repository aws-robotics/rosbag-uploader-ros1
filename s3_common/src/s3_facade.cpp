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

#include <aws/core/utils/logging/LogMacros.h>
#include <aws/s3/S3Client.h>

#include <s3_common/s3_common_error_codes.h>
#include <s3_common/s3_facade.h>

namespace Aws
{
namespace S3
{

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
    AWS_LOG_INFO(__func__, "Upload: %s to s3://%s/%s", file_path.c_str(), bucket.c_str(), key.c_str());
    return Aws::S3::S3ErrorCode::SUCCESS;
}


}  // namespace S3
}  // namespace Aws
