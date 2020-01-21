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
namespace Rosbag
{

/**
 * Error codes for S3.
 */
enum RecorderErrorCode
{
  // Operation was successfuly
  SUCCESS = 0,
  // Generic failure
  FAILED,
  // The local rosgbag file was not found
  FILE_NOT_FOUND,
  // Fail to remove local rosgbag file
  FILE_REMOVAL_FAILED,
  // recorder is not recording
  RECORDER_NOT_RUNNING,
  // recorder is already started recording
  RECORDER_IS_RUNNING,
};

}  // namespace Rosbag
}  // namespace Aws
