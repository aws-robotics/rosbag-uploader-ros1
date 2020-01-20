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
#include <ros/ros.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rosbag_rolling_recorder");
  Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("RosbagRollingRecord"));

  // TODO(abbyxu): will remove in subsequent PRs
  ros::Duration bag_rollover_time(10);
  ros::Duration max_record_time(30);
  std::string write_dir("~/.ros/rosbag_uploader");
  AWS_LOG_INFO(__func__, "Starting rolling recorder node.");

  Aws::Rosbag::RollingRecorder rolling_recorder(bag_rollover_time, max_record_time, write_dir);

  ros::waitForShutdown();
  AWS_LOG_INFO(__func__, "Finishing rolling recorder node.");
  Aws::Utils::Logging::ShutdownAWSLogging();
  return 0;
}
