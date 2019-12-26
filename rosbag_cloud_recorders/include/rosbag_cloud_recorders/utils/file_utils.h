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

#include <string>

#include <rosbag_cloud_recorders/recorder_common_error_codes.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

  /**
  * @brief delete a file
  *
  * Delete file at file_path.
  *
  * @param file_path path to the file to be deleted
  * @return error code, SUCCESS if the file is sucessfully deleted
  */
Aws::Rosbag::RecorderErrorCode DeleteFile(const std::string & file_path);

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
