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
#include <rolling_recorder/rolling_recorder.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>

int main(int argc, char* argv[])
{
  Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("RosbagRollingRecord"));
  ros::init(argc, argv, "rosbag_rolling_recorder");
  ros::spin();
  return 0;
}
