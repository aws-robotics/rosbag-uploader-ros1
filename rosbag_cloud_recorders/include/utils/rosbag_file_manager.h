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

#include <recorder_common_error_codes.h>
#include <string>
#include <fstream>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

class RosbagFileManager
{
 public:
  /**
   * Default Constructor.
   */
  RosbagFileManager() = default;
  ~RosbagFileManager() = default;

  /**
  * @brief delete a rosbag file that has alreaady been uploaded to an Amazon S3 bucket
  *
  * Delete file at rosbag_file_path.
  *
  * @param rosbag_file_path path to the rosbag file to be deleted
  * @return error code, SUCCESS if the file is sucessfully deleted
  */
  Aws::Rosbag::RecorderErrorCode DeleteRosbag(const std::string & rosbag_file_path);
};

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
