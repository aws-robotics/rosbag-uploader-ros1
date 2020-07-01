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

#include <cerrno>
#include <cstring>
#include <unistd.h>

#include <array>
#include <exception>
#include <functional>
#include <iostream>
#include <regex>
#include <string>

#include <boost/date_time/c_local_time_adjustor.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/date_time/local_time_adjustor.hpp>
#include <boost/filesystem.hpp>

#include <aws/core/utils/logging/LogMacros.h>
#include <aws_common/fs_utils/wordexp_ros.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <rosbag_cloud_recorders/recorder_common_error_codes.h>

namespace
{

constexpr int kMaxErrStrSize = 256;
constexpr char kRosBagFileFormat[] = "%Y-%m-%d-%H-%M-%S";

}

namespace Aws
{
namespace Rosbag
{
namespace Utils
{


bool ExpandAndCreateDir(const std::string & dir, std::string & expanded_dir)
{
  wordexp_t wordexp_result{};

  int expand_result = wordexp_ros(dir.c_str(), &wordexp_result, 0);
  // Directory was successfully read and expanded
  if (0 == expand_result && 1 == wordexp_result.we_wordc) {
    expanded_dir = *(wordexp_result.we_wordv);
  } else {
    std::array<char, kMaxErrStrSize> err_msg{};
    (void) strerror_r(errno, err_msg.data(), err_msg.max_size());
    AWS_LOGSTREAM_ERROR(__func__, "Failed to expand write directory " << expanded_dir << " with error " << err_msg.data());
    wordfree(&wordexp_result);
    return false;
  }

  if (!boost::filesystem::exists(expanded_dir)) {
    AWS_LOGSTREAM_INFO(__func__, "Provided write directory " << expanded_dir << " doesn't exist, creating.");
    boost::filesystem::create_directories(expanded_dir);
    if (!boost::filesystem::exists(expanded_dir)) {
      AWS_LOGSTREAM_ERROR(__func__, "Failed to create write directory " << expanded_dir);
      wordfree(&wordexp_result);
      return false;
    }
  }

  wordfree(&wordexp_result);

  int writeable_result = access(expanded_dir.c_str(), W_OK);  // test for writeability
  if (0 != writeable_result) {
    AWS_LOGSTREAM_WARN(__func__, "Provided write directory " << expanded_dir << " is not writeable");
  }

  return boost::filesystem::is_directory(expanded_dir) && 0 == writeable_result;
}

Aws::Rosbag::RecorderErrorCode DeleteFile(const std::string & file_path)
{
  const int result = unlink(file_path.c_str());
  if (result == 0) {
    AWS_LOGSTREAM_INFO(__func__, "Deleted file " << file_path);
    return Aws::Rosbag::RecorderErrorCode::SUCCESS;
  } else {
    std::array<char, kMaxErrStrSize> err_msg{};
    (void) strerror_r(errno, err_msg.data(), err_msg.max_size());
    if (errno == ENOENT) {
      AWS_LOGSTREAM_WARN(__func__, "Failed to delete file: " << file_path << ' ' << err_msg.data());
      return Aws::Rosbag::RecorderErrorCode::FILE_NOT_FOUND;
    }
    AWS_LOGSTREAM_ERROR(__func__, "Failed to delete file: " << file_path << ' ' << err_msg.data());
    return Aws::Rosbag::RecorderErrorCode::FILE_REMOVAL_FAILED;
  }
}

boost::posix_time::time_duration GetUTCOffset() {
    using local_adj = boost::date_time::c_local_adjustor<boost::posix_time::ptime>;

    const boost::posix_time::ptime utc_now = boost::posix_time::second_clock::universal_time();
    const boost::posix_time::ptime now = local_adj::utc_to_local(utc_now);
    return now - utc_now;
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
  std::string ts_unparsed = bag_name;
  index = file_path.find_last_of('.');
  if (index != std::string::npos) {
    ts_unparsed = bag_name.substr(0, index);
  }
  
  // Pull the timestamp out of the remaining string
  std::regex time_stamp_regex(R"([0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2})");
  std::smatch match;
  auto ts_begin = std::sregex_iterator(ts_unparsed.begin(), ts_unparsed.end(), time_stamp_regex);
  auto ts_end = std::sregex_iterator();
  while (ts_begin != ts_end) {
    match = *ts_begin;
    ts_begin++;
  }
  if (match.empty()) {
    AWS_LOGSTREAM_WARN(__func__, "Could not find timestamp in rosbag filename via regex");
    return {};
  }
  std::string time_stamp = match.str(0);

  // Convert time stamp to ros time
  auto input_facet = new boost::posix_time::time_input_facet(kRosBagFileFormat);
  std::stringstream ss;
  ss.imbue(std::locale(ss.getloc(), input_facet));
  ss.str(time_stamp);
  boost::posix_time::ptime pt;
  ss >> pt;

  if (pt == boost::posix_time::ptime()) {
    AWS_LOGSTREAM_WARN(__func__, "Parsing rosbag file timestamp failed");
    return {};
  }

  boost::posix_time::ptime utc_pt = pt - GetUTCOffset();

  try {
    // This can throw an exception if the time is too far in the future.
    return ros::Time::fromBoost(utc_pt);
  } catch (std::exception& e) {
    AWS_LOGSTREAM_WARN(__func__, "Parsing rosbag file timestamp threw exception: " << e.what());
    return {};
  }
}

std::vector<std::string> GetRosbagsToUpload(const std::string& search_directory, const std::function<bool (rosbag::View&)>& select_file)
{
  std::vector<std::string> ros_bags_to_upload;
  using boost::filesystem::directory_iterator;
  boost::filesystem::path ros_bag_search_path(search_directory);
  for (auto dir_entry = directory_iterator(ros_bag_search_path); dir_entry != directory_iterator(); dir_entry++) {
    if (boost::filesystem::is_directory(dir_entry->path())) {
      continue;
    }
    if (dir_entry->path().extension().string() == ".bag") {
      rosbag::Bag ros_bag;
      ros_bag.open(dir_entry->path().string());
      rosbag::View view_rosbag(ros_bag);
      if (select_file(view_rosbag)){
        ros_bags_to_upload.push_back(dir_entry->path().string());
        AWS_LOG_INFO(__func__, "Adding bag: [%s] to list of bag files to upload.", dir_entry->path().string().c_str());
      }
      ros_bag.close();
    }
  }
  return ros_bags_to_upload;
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
