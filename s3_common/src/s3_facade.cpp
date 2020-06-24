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
#include <s3_common/utils.h>

namespace Aws
{
namespace S3
{

S3Facade::S3Facade(const bool enable_encryption)
: S3Facade(enable_encryption, std::make_unique<S3Client>())
{
}

S3Facade::S3Facade(const bool enable_encryption, const Aws::Client::ClientConfiguration & config)
: S3Facade(enable_encryption, std::make_unique<S3Client>(config))
{
}

S3Facade::S3Facade(const bool enable_encryption, std::unique_ptr<S3Client> s3_client)
: s3_client_(std::move(s3_client)), enable_encryption_(enable_encryption)
{
}

Model::PutObjectOutcome S3Facade::PutObject(
  const std::string & file_path,
  const std::string & bucket,
  const std::string & key)
{
  AWS_LOGSTREAM_INFO(__func__, "Upload: "<<file_path<<" to s3://"<<bucket<<"/"<<key);
  const std::shared_ptr<Aws::IOStream> file_data = 
      std::make_shared<Aws::FStream>(file_path.c_str(),
                       std::ios_base::in | std::ios_base::binary);
  if (!file_data->good()) {
    AWS_LOGSTREAM_ERROR(__func__, "Upload aborted, file " << file_path << " couldn't be opened for reading");
    Aws::StringStream result;
    result << "File " << file_path << " couldn't be opened for reading";
    return Model::PutObjectOutcome(Aws::Client::AWSError<S3Errors>(S3Errors::INVALID_PARAMETER_VALUE,
                     "INVALID_PARAMETER_VALUE", Aws::String(result.str()), false));
  }
  Aws::S3::Model::PutObjectRequest put_object_request;
  put_object_request.SetBucket(bucket.c_str());
  put_object_request.SetKey(key.c_str());
  put_object_request.SetBody(file_data);
  if (enable_encryption_) {
    put_object_request.SetServerSideEncryption(Aws::S3::Model::ServerSideEncryption::AES256);
  } else {
    put_object_request.SetServerSideEncryption(Aws::S3::Model::ServerSideEncryption::NOT_SET);
  }

  auto outcome = s3_client_->PutObject(put_object_request);
  if (outcome.IsSuccess()) {
    AWS_LOGSTREAM_INFO(__func__, "Successfully uploaded " << file_path << " to s3://" << bucket << "/" << key);
  } else {
    const auto & error = outcome.GetError();
    AWS_LOGSTREAM_ERROR(__func__, "Failed to upload " << file_path << " to s3://" << bucket << "/" << key
                                  << ": " << error.GetMessage());
  }
  return outcome;
}

}  // namespace S3
}  // namespace Aws
