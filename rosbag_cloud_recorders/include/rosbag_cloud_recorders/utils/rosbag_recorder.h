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

#include <chrono>
#include <functional>
#include <future>
#include <mutex>
#include <thread>

#include <aws/core/utils/logging/LogMacros.h>

#include <rosbag_cloud_recorders/utils/recorder.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

enum RosbagRecorderRunResult
{
  // Started the rosbag recorder successfully
  STARTED = 0,
  // Rosbag recorder not run as it may be running already
  SKIPPED
};

/*
 * Wrapper class around robag::Recorder that allows for executing callback
 * functions before and after collecting rosbag files for the specified
 * duration.
 * 
 * rosbag::Recorder has non-virtual members that it impractical to extend and
 * therefore hard to mock. RosbagRecorder is therefore wrapping rosbag::recorder
 * and is also templatized to allow for injection based mocking.
 */
template<typename T>
class RosbagRecorder
{
public:
  explicit RosbagRecorder()
  {
    std::promise<void> p;
    barrier_ = p.get_future();
  };

  virtual ~RosbagRecorder() = default;

  virtual RosbagRecorderRunResult Run(
    const rosbag::RecorderOptions& recorder_options,
    const std::function<void()>& pre_record,
    const std::function<void(int)>& post_record
  )
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (IsActive()) {
        AWS_LOG_INFO(__func__, "Failed to run RosbagRecorder, recorder already active");
        return RosbagRecorderRunResult::SKIPPED;
      }
      AWS_LOGSTREAM_INFO(__func__, "compression " << recorder_options.compression);
      AWS_LOG_INFO(__func__, "Starting a new RosbagRecorder session");
      static auto function_name = __func__;
      barrier_ = std::async(std::launch::async, [recorder_options, pre_record, post_record]
        {
          pre_record();
          T rosbag_recorder(recorder_options);
          int exit_code = rosbag_recorder.run();
          if (exit_code != 0) {
            AWS_LOG_ERROR(function_name, "RosbagRecorder encountered an error");
          }
          post_record(exit_code);
        }
      );
      return RosbagRecorderRunResult::STARTED;
    }
  }

  virtual bool IsActive() const
  {
    using namespace std::chrono_literals;
    auto status = barrier_.wait_for(0ms);
    return std::future_status::ready != status;
  }

private:
  mutable std::mutex mutex_;
  std::future<void> barrier_;
};

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
