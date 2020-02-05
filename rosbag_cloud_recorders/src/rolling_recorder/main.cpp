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
#include <aws_common/fs_utils/wordexp_ros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>

constexpr char kBagRolloverTimeParameter[] = "bag_rollover_time";
constexpr char kMaxRecordTimeParameter[] = "max_record_time";
constexpr char kWriteDirectoryParameter[] = "write_directory";

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rosbag_rolling_recorder");
  Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("RosbagRollingRecord"));

  int bag_rollover_time_input;
  int max_record_time_input;
  std::string write_directory_input;

  ros::Duration bag_rollover_time(30);
  ros::Duration max_record_time(300);
  std::string write_dir("~/.ros/rosbag_uploader");

  auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kBagRolloverTimeParameter), bag_rollover_time_input)) {
    bag_rollover_time = ros::Duration(bag_rollover_time_input);
  }
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kMaxRecordTimeParameter), max_record_time_input)) {
    max_record_time = ros::Duration(max_record_time_input);
  }
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kWriteDirectoryParameter), write_directory_input)) {
    write_dir = write_directory_input;
  }

  wordexp_t wordexp_result;
  wordexp_ros(write_dir, &wordexp_result, 0);
  AWS_LOG_INFO(__func__, "Starting rolling recorder node.");
  Aws::Rosbag::RollingRecorder rolling_recorder(bag_rollover_time, max_record_time, *(wordexp_result.we_wordv));
  ros::waitForShutdown();
  AWS_LOG_INFO(__func__, "Finishing rolling recorder node.");
  Aws::Utils::Logging::ShutdownAWSLogging();
  return 0;
}
