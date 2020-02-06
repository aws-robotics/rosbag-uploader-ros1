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
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>

#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>

using namespace Aws::Rosbag::Utils;

class MockRecorder
{
public:
  MockRecorder(RecorderOptions const& options) {
    (void) options;
  };
  ~MockRecorder() {};

  int Run()
  {
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
    return 0;
  }
};

TEST(TestRosbagRecorder, TestRosbagRecorderRun)
{
  RecorderOptions options;
  RosbagRecorder<MockRecorder> rosbag_recorder;
 
  ASSERT_FALSE(rosbag_recorder.IsActive());
  
  std::promise<bool> pre_invoked;
  std::promise<bool> post_invoked;
  
  std::future<bool> pre_invoked_f = pre_invoked.get_future();
  std::future<bool> post_invoked_f = post_invoked.get_future();
  auto result = rosbag_recorder.Run(
    options,
    [&]
    {
      pre_invoked.set_value(true);
    },
    [&](int exit_code)
    {
      ASSERT_EQ(exit_code, 0);
      post_invoked.set_value(true);
    }
  );
  ASSERT_EQ(result, RosbagRecorderRunResult::STARTED);
  ASSERT_TRUE(pre_invoked_f.get());
  ASSERT_TRUE(post_invoked_f.get());
}

TEST(TestRosbagRecorder, TestRosbagRecorderIsActive)
{
  RosbagRecorder<MockRecorder> rosbag_recorder;
  RecorderOptions options;
  
  std::promise<void> barrier;
  std::future<void> barrier_future = barrier.get_future();
  
  ASSERT_FALSE(rosbag_recorder.IsActive());

  auto result = rosbag_recorder.Run(
    options,
    []
    {
    },
    [&](int exit_code)
    {
      ASSERT_EQ(exit_code, 0);
      barrier_future.wait();
    }
  );

  ASSERT_EQ(result, RosbagRecorderRunResult::STARTED);
  ASSERT_TRUE(rosbag_recorder.IsActive());

  result = rosbag_recorder.Run(
    options,
    []
    {
    },
    [&](int exit_code)
    {
      (void) exit_code;
    }
  );

  ASSERT_EQ(result, RosbagRecorderRunResult::SKIPPED);

  barrier.set_value();

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1s);

  ASSERT_FALSE(rosbag_recorder.IsActive());
}
