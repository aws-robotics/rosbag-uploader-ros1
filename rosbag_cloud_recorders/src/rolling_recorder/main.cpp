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

#include <string>
#include <vector>
#include <ros/ros.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <thread>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rosbag_rolling_recorder");
  Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("RosbagRollingRecord"));

  ros::Duration bag_rollover_time(10);
  ros::Duration max_record_time(30);
  std::vector<std::string> topics_to_record;
  topics_to_record.emplace_back("rosout_agg");

  AWS_LOG_INFO(__func__, "Starting rolling recorder node.");

  ros::AsyncSpinner executor(0);
  executor.start();

  Aws::Rosbag::RollingRecorder rolling_recorder(bag_rollover_time, max_record_time, topics_to_record);
  std::thread start_rolling_recorder( [&rolling_recorder] { rolling_recorder.StartRollingRecorder(); } );

  ros::waitForShutdown();
  executor.stop();
  start_rolling_recorder.join();
  AWS_LOG_INFO(__func__, "Finishing rolling recorder node.");
  Aws::Utils::Logging::ShutdownAWSLogging();
  return 0;
}
