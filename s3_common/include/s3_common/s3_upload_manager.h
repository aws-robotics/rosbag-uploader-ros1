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

#include <aws/s3/S3Client.h>

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
  bool operator==(const UploadDescription& rhs) const
  {
    return file_path == rhs.file_path && object_key == rhs.object_key;
  }
};

// Manages uploading a list of files to Amazon S3
class S3UploadManager
{
public:
 /**
  * Creates an S3UploadManager with a default-constructed S3Facade
  *
  * @param enable_encryption whether or not to enable AES256 server-side encryption
  */
  explicit S3UploadManager(const bool enable_encryption);
  
 /**
  * Creates an S3UploadManager with a S3Facade that uses the given ClientConfiguration
  *
  * @param enable_encryption whether or not to enable AES256 server-side encryption
  * @param config the ClientConfiguration to be used
  */
  explicit S3UploadManager(const bool enable_encryption, const Aws::Client::ClientConfiguration &config);

 /**
  * Creates an S3UploadManager with the given existing S3Facade
  *
  * @param s3_facade the existing S3Facade to be used
  */
  explicit S3UploadManager(std::unique_ptr<S3Facade> s3_facade);

  virtual ~S3UploadManager() = default;

  /* Cancel the current upload
   */
  virtual void CancelUpload();

  /* Upload a list of files to S3
   * @param upload_descriptions a vector of files to upload to S3
   * @param bucket the name of the s3 bucket to target. Must be in the same region as the client config
   * @param feedback_callback called with the list of UploadDescriptions that have been finished
   * @return An PutObjectOutcome
   */ 
  virtual Model::PutObjectOutcome UploadFiles(
    const std::vector<UploadDescription> & upload_descriptions,
    const std::string & bucket,
    const boost::function<void (const std::vector<UploadDescription>&)>& feedback_callback);

  virtual bool IsAvailable() const;

private:
  // The current state of the upload manager
  S3UploadManagerState manager_status_;
  // Guards manager_status_
  mutable std::recursive_mutex mutex_;
  // Facade for interaction with S3 client
  std::unique_ptr<S3Facade> s3_facade_;
};

}  // namespace S3
}  // namespace Aws
