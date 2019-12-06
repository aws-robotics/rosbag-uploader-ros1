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

namespace Aws
{
namespace S3
{

/**
 * Error codes for S3.
 */
enum S3ErrorCode
{
  // Operation was successfuly
  SUCCESS = 0,
  // Upload was cancelled
  CANCELLED,
  // Upload rejected because another upload request was active
  UPLOADER_BUSY,
  // Generic failure
  FAILED,
  // The local file was not found or couldn't be opened
  FILE_COULDNT_BE_READ,
  // Access to S3 resources was denied
  S3_ACCESS_DENIED,
  // S3 bucket doesn't exist
  S3_NO_SUCH_BUCKET
};

}  // namespace S3
}  // namespace Aws
