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
#include <cstring>
#include <cerrno>
#include <string>
#include <unistd.h>

#include <aws/core/utils/logging/LogMacros.h>

#include <rosbag_cloud_recorders/recorder_common_error_codes.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

Aws::Rosbag::RecorderErrorCode DeleteFile(const std::string & file_path)
{
  const int result = unlink(file_path.c_str());
  if (result == 0) {
    AWS_LOGSTREAM_INFO(__func__, "Deleted file "<<file_path);
    return Aws::Rosbag::RecorderErrorCode::SUCCESS;
  } else {
    if (errno == ENOENT) {
      AWS_LOGSTREAM_WARN(__func__, "Failed to delete file: "<<file_path<<" "<<std::strerror(errno));
      return Aws::Rosbag::RecorderErrorCode::FILE_NOT_FOUND;
    }
    AWS_LOGSTREAM_ERROR(__func__, "Failed to delete file: "<<file_path<<" "<<std::strerror(errno));
    return Aws::Rosbag::RecorderErrorCode::FILE_REMOVAL_FAILED;
  }
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
