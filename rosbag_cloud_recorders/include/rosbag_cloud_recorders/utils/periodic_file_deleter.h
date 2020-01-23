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

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <boost/function.hpp>
#include <ros/ros.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

class PeriodicFileDeleter
{
public:
  explicit PeriodicFileDeleter(
    boost::function<std::vector<std::string>()> deletion_list_callback,
    const int interval_period_s=10);
  ~PeriodicFileDeleter();
  // Delete copy and move constructors
  PeriodicFileDeleter(PeriodicFileDeleter const&) = delete;
  PeriodicFileDeleter& operator =(PeriodicFileDeleter const&) = delete;
  PeriodicFileDeleter(PeriodicFileDeleter&&) = delete;
  PeriodicFileDeleter& operator=(PeriodicFileDeleter&&) = delete;

  void Start();
  void Stop();
  bool IsActive() const;
private:
  // Whether the deleter should be deleting files now
  bool is_active_;
  // Callback function to get a list of files for deletion
  boost::function<std::vector<std::string>()> deletion_list_callback_;
  // How often the deleter should check for new files to delete
  const int interval_period_s_;
  // Protects is_active_
  mutable std::mutex mutex_;
  // Runs the deletion loop
  std::thread thread_;
  // Main logic for deleting a list of files
  void DeleteFiles();
};

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws