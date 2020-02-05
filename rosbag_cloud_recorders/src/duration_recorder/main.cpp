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
#include <ros/ros.h>

#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>

#include <rosbag_cloud_recorders/duration_recorder/duration_recorder.h>

constexpr char kNodeName[] = "rosbag_duration_recorder";

int main(int argc, char* argv[])
{
  ros::init(argc, argv, kNodeName);
  Aws::Utils::Logging::InitializeAWSLogging(
        Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));

  Aws::Rosbag::DurationRecorderOptions duration_recorder_options;
  duration_recorder_options.write_directory = "/tmp/";
  AWS_LOG_INFO(__func__, "Starting duration recorder");

  Aws::Rosbag::DurationRecorder duration_recorder(duration_recorder_options);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  Aws::Utils::Logging::ShutdownAWSLogging();
  
  return 0;
}
