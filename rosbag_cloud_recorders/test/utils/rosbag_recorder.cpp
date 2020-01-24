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
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <atomic>

#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>

using namespace Aws::Rosbag::Utils;

class MockRecorder
{
public:
  MockRecorder() {
    invocation_count_.store(0);
  };
  ~MockRecorder() {};

  int get_invocation_count() {
    return invocation_count_.load();
  }

  void run()
  {
    invocation_count_++;
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
  }
protected: 
  std::atomic<int> invocation_count_;
};

TEST(TestRosbagRecorder, TestRosbagRecorderRun)
{
 
  MockRecorder recorder;
  RosbagRecorder<MockRecorder> rosbag_recorder(recorder);
 
  ASSERT_FALSE(rosbag_recorder.IsActive());
  
  std::promise<bool> pre_invoked;
  std::promise<bool> post_invoked;
  
  std::future<bool> pre_invoked_f = pre_invoked.get_future();
  std::future<bool> post_invoked_f = post_invoked.get_future();
  rosbag_recorder.Run(
    [&]
    {
      std::promise<bool> invoked = std::move(pre_invoked);
      invoked.set_value(true);
    },
    [&]
    {
      std::promise<bool> invoked = std::move(post_invoked);
      invoked.set_value(true);
    }
  );

  ASSERT_TRUE(pre_invoked_f.get());
  ASSERT_TRUE(post_invoked_f.get());
}

TEST(TestRosbagRecorder, TestRosbagRecorderIsActive)
{
  MockRecorder recorder;
  RosbagRecorder<MockRecorder> rosbag_recorder(recorder);
  
  std::promise<void> barrier;
  std::future<void> barrier_future = barrier.get_future();
  
  std::promise<void> barrier_second_invoke;
  std::future<void> barrier_second_invoke_future = barrier_second_invoke.get_future();
  
  std::promise<bool> invoked;
  std::future<bool> invoked_future = invoked.get_future();
  
  std::promise<bool> second_invoked;
  std::future<bool> second_invoked_future = second_invoked.get_future();
  
  ASSERT_FALSE(rosbag_recorder.IsActive());
  
  rosbag_recorder.Run(
    []
    {
    },
    [&]
    {
      barrier_future.wait();
      invoked.set_value(true);
    }
  );
  
  ASSERT_TRUE(rosbag_recorder.IsActive());
 
  rosbag_recorder.Run(
    []
    {
    },
    [&]
    {
      barrier_second_invoke_future.wait();
      second_invoked.set_value(true);
    }
  );
  barrier.set_value();
  invoked_future.get();
  
  barrier_second_invoke.set_value();
  std::future_status status = second_invoked_future.wait_for(
    std::chrono::seconds(1)
  );
  ASSERT_NE(std::future_status::ready, status);
  ASSERT_EQ(1, recorder.get_invocation_count());
}