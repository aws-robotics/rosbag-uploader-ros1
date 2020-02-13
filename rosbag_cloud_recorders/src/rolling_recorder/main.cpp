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
constexpr char kNodeName[] = "rosbag_rolling_recorder";
constexpr char kUploadTimeoutParameter[] = "upload_timeout_s";
constexpr char kWriteDirectoryDefault[] = "~/.ros/rr_rosbag_uploader/";
constexpr uint32_t kTimeOutDefaultInSeconds = 3600;
constexpr uint32_t kBagRolloverTimeDefaultInSeconds = 30;
constexpr uint32_t kMaxRecordTimeDefaultInSeconds = 300;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, kNodeName);
  Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));

  Aws::Rosbag::RollingRecorderOptions rolling_recorder_options;
  std::string write_directory_input;
  int max_record_time_input;
  int bag_rollover_time_input;

  auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();
  // Set bag_rollover_time
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kBagRolloverTimeParameter), bag_rollover_time_input)) {
    rolling_recorder_options.bag_rollover_time = ros::Duration(bag_rollover_time_input);
  } else {
    rolling_recorder_options.bag_rollover_time = ros::Duration(kBagRolloverTimeDefaultInSeconds);
  }

  // Set max_record_time
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kMaxRecordTimeParameter), max_record_time_input)) {
    rolling_recorder_options.max_record_time = ros::Duration(max_record_time_input);
  } else {
    rolling_recorder_options.max_record_time = ros::Duration(kMaxRecordTimeDefaultInSeconds);
  }

  // Set operation time out in seconds
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kUploadTimeoutParameter), rolling_recorder_options.upload_timeout_s)) {
    rolling_recorder_options.upload_timeout_s = kTimeOutDefaultInSeconds;
  }

  // Set write_directory
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kWriteDirectoryParameter), write_directory_input)) {
    write_directory_input = kWriteDirectoryDefault;
  }


  wordexp_t wordexp_result;
  wordexp_ros(write_directory_input.c_str(), &wordexp_result, 0);
  rolling_recorder_options.write_directory = *(wordexp_result.we_wordv);

  AWS_LOG_INFO(__func__, "Starting rolling recorder node.");
  Aws::Rosbag::RollingRecorder rolling_recorder(rolling_recorder_options);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  AWS_LOG_INFO(__func__, "Finishing rolling recorder node.");
  Aws::Utils::Logging::ShutdownAWSLogging();
  return 0;
}
