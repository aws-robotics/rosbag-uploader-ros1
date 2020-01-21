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
#include <exception>
#include <string>
#include <unistd.h>
#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <aws/core/utils/logging/LogMacros.h>
#include <ros/ros.h>

#include <rosbag_cloud_recorders/recorder_common_error_codes.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

constexpr char kRosBagFileFormat[] = "%Y-%m-%d-%H-%M-%S";

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


ros::Time GetRosBagStartTime(const std::string& file_path)
{
  // Get the file name
  std::string file_name = file_path;
  size_t index = file_path.find_last_of('/');
  if (index != std::string::npos) {
    file_name = file_path.substr(index+1);
  }

  // Strip bag number if it exists
  std::string bag_name = file_name;
  index = file_path.find_last_of('_');
  if (index != std::string::npos) {
    bag_name = file_name.substr(0, index);
  }

  // If bag extension wasn't stripped before, remove it now
  std::string time_stamp = bag_name;
  index = file_path.find_last_of('.');
  if (index != std::string::npos) {
    time_stamp = bag_name.substr(0, index);
  }

  // Convert time stamp to ros time
  auto input_facet = new boost::posix_time::time_input_facet(kRosBagFileFormat);
  std::stringstream ss;
  ss.imbue(std::locale(ss.getloc(), input_facet));
  ss.str(time_stamp);
  boost::posix_time::ptime pt;
  
  ss >> pt;
  if (pt == boost::posix_time::ptime()) {
    AWS_LOGSTREAM_WARN(__func__, "Parsing rsobag file timestamp failed");
    return {};
  }
  try {
    // This can throw an exception if the time is too far in the future.
    return ros::Time::fromBoost(pt);
  } catch (std::exception& e) {
    AWS_LOGSTREAM_WARN(__func__, "Parsing rosbag file timestamp threw exception: " << e.what());
    return {};
  }
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
