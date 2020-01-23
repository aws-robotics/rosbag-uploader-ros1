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

#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <aws/core/utils/logging/LogMacros.h>
#include <boost/function.hpp>
#include <ros/ros.h>

#include <rosbag_cloud_recorders/recorder_common_error_codes.h>
#include <rosbag_cloud_recorders/utils/file_utils.h>
#include <rosbag_cloud_recorders/utils/periodic_file_deleter.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

PeriodicFileDeleter::PeriodicFileDeleter(
  boost::function<std::vector<std::string> ()> deletion_list_callback,
  const int interval_period_s)
: is_active_(false),
deletion_list_callback_(std::move(deletion_list_callback)),
interval_period_s_(interval_period_s)
{}

PeriodicFileDeleter::~PeriodicFileDeleter()
{
  Stop();
}

void PeriodicFileDeleter::Start()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_active_) {
      AWS_LOG_INFO(__func__, "Failed to start PeriodicFileDeleter, deleter already active");
      return;
    }
    is_active_ = true;
  }
  thread_ = std::thread{&PeriodicFileDeleter::DeleteFiles, this};
  AWS_LOG_INFO(__func__, "PeriodicFileDeleter started");
}

void PeriodicFileDeleter::Stop()
{
  AWS_LOG_INFO(__func__, "Stopping PeriodicFileDeleter");
  std::lock_guard<std::mutex> lock(mutex_);
  is_active_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool PeriodicFileDeleter::IsActive() const
{
  return is_active_;
}

void PeriodicFileDeleter::DeleteFiles()
{
  while (is_active_) {
    auto files_to_delete = deletion_list_callback_();
    for (const auto& file: files_to_delete) {
      AWS_LOGSTREAM_INFO(__func__, "Deleting file " << file);
      auto result = DeleteFile(file);
      if (result != RecorderErrorCode::SUCCESS) {
        AWS_LOGSTREAM_ERROR(__func__, "Failed to delete file " << file << ", skipping.");
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(interval_period_s_));
  }
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws