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

#include <aws/s3/S3Client.h>

namespace Aws
{
namespace S3
{

class S3Facade
{
public:
  // Creates an S3Client with default ClientConfig
  S3Facade();
  S3Facade(const S3Facade & other) = delete;
  // Provide the ClientConfiguration used to create the S3Client
  explicit S3Facade(const Aws::Client::ClientConfiguration& config);
  explicit S3Facade(std::unique_ptr<Aws::S3::S3Client> s3_client);
  virtual ~S3Facade() = default;

  S3Facade & operator=(const S3Facade & other) = delete;

  /**
  * @brief Enable server-side encryption of uploaded files
  *
  * Specifies whether or not the uploaded files should be stored encrypted on S3.
  *
  * @param enable whether or not to enable encryption
  */
  void EnableEncryption(const bool enable);

  /**
  * @brief Call s3 PutObject api to upload file to s3
  *
  * Synchronous call to S3. Uploads file at file_path to s3.
  *
  * @param file_path path to the file to upload
  * @param bucket the s3 bucket for upload
  * @param key object key for upload
  * @return S3 PutObjectOutcome describing result of upload,
  */
  virtual Model::PutObjectOutcome PutObject(const std::string& file_path, const std::string& bucket, const std::string& key);

private:
  Aws::Client::ClientConfiguration config_;
  std::unique_ptr<Aws::S3::S3Client> s3_client_;
  bool enable_encryption_ = false;
};

}  // namespace S3
}  // namespace Aws
