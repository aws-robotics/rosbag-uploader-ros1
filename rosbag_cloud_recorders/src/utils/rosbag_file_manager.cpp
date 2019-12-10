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

#include <recorder_common_error_codes.h>
#include <utils/rosbag_file_manager.h>
#include <string>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

Aws::Rosbag::RecorderErrorCode RosbagFileManager::DeleteRosbag(const std::string & rosbag_file_path)
{
  std::ifstream rosbag_file(rosbag_file_path);
  if (!rosbag_file.good())
  {
    return Aws::Rosbag::RecorderErrorCode::ROSBAG_FILE_NOT_FOUND;
  }

  if (std::remove(rosbag_file_path.c_str()) == 0) {
    return Aws::Rosbag::RecorderErrorCode::SUCCESS;
  }
  return Aws::Rosbag::RecorderErrorCode::ROSBAG_REMOVAL_FAILED;
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
