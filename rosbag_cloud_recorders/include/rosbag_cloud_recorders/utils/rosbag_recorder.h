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

#include <thread>
#include <functional>

#include <aws/core/utils/logging/LogMacros.h>

#include <rosbag/recorder.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{
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
  explicit RosbagRecorder() = default;
  // Non-copyable. These definitions are required as we have 
  // non-default destructor.
  RosbagRecorder(const RosbagRecorder&) = delete;
  RosbagRecorder& operator=(const RosbagRecorder&) = delete;
  
  virtual ~RosbagRecorder()
  {
    if (runner_thread_.joinable())
    {
      runner_thread_.join();
    }
  }

  virtual void Run(
    rosbag::RecorderOptions const& recorder_options,
    const std::function<void()>& pre_record,
    const std::function<void()>& post_record
  )
  {
    if (IsActive()) {
      AWS_LOG_INFO(__func__, "Failed to run RosbagRecorder, recorder already active");
      return;
    }
    AWS_LOG_INFO(__func__, "Starting a new RosbagRecorder session");
    runner_thread_ = std::thread([recorder_options, pre_record, post_record]
      {
        pre_record();
        T rosbag_recorder(recorder_options);
        rosbag_recorder.run();
        post_record();
      }
    );
  }

  virtual bool IsActive() const
  {
     return runner_thread_.joinable();
  }

private:
  std::thread runner_thread_;
};

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
