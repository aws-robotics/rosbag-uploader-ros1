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

#include <aws/core/utils/logging/LogMacros.h>

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
Aws::Rosbag::RecorderErrorCode DeleteFile(const std::string & file_path)
{
  std::ifstream file(file_path);
  if (!file.good()) {
    AWS_LOGSTREAM_ERROR(__func__, "Failed to delete file: " << file_path << ", file not found.");
    return Aws::Rosbag::RecorderErrorCode::FILE_NOT_FOUND;
  }
  if (std::remove(file_path.c_str()) == 0) {
    AWS_LOGSTREAM_DEBUG(__func__, "Deleted file " << file_path);
    return Aws::Rosbag::RecorderErrorCode::SUCCESS;
  }
  AWS_LOGSTREAM_ERROR(__func__, "Failed to delete file: " << file_path);
  return Aws::Rosbag::RecorderErrorCode::FILE_REMOVAL_FAILED;
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
