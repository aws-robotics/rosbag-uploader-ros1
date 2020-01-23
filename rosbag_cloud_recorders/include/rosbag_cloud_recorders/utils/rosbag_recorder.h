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
#pragma once

#include <atomic>
#include <functional>

#include <rosbag/recorder.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

class RosbagRecorder
{
public:
  explicit RosbagRecorder(rosbag::RecorderOptions const& options);
  ~RosbagRecorder();

  virtual void Run(std::function<void()> callback);
  virtual bool IsActive() const;
private:
  rosbag::Recorder rosbag_recorder_;
  std::atomic<bool> running_;
};

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
