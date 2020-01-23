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

void RosbagRecorder::Run(std::function<void()> callback)
{
  if (IsActive()) {
    AWS_LOG_INFO(__func__, "Failed to run RosbagRecorder, recorder already active");
    return;
  }
  AWS_LOG_INFO(__func__, "Starting a new RosbagRecorder session");
  running_.store(true);
  std::thread([this, callback] {
    this->rosbag_recorder_.run();
    this->running_.store(false);
    callback();
  });
}

bool RosbagRecorder::IsActive() const
{
  return running_.load();
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws