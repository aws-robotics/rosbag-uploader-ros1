/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <future>
#include <thread>
#include <chrono>

#include <aws/core/utils/logging/LogMacros.h>

#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

RosbagRecorder::~RosbagRecorder()
{
}

void RosbagRecorder::Run(std::function<void()> pre_record, std::function<void()> post_record)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_active_) {
      AWS_LOG_INFO(__func__, "Failed to run RosbagRecorder, recorder already active");
      return;
    }
    AWS_LOG_INFO(__func__, "Starting a new RosbagRecorder session");
    is_active_ = true;
    std::thread([this, pre_record, post_record]
      {
        pre_record();
        this->rosbag_recorder_.run();
        this->is_active_ = false;
        post_record();
      }
    );
  }
}

bool RosbagRecorder::IsActive() const
{
  return is_active_;
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws