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
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_common/fs_utils/wordexp_ros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>

#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder.h>
#include <rosbag_cloud_recorders/utils/file_utils.h>
#include <rosbag_cloud_recorders/utils/recorder.h>
#include <rosbag_cloud_recorders/utils/rosbag_recorder.h>

constexpr char kNodeName[] = "rosbag_rolling_recorder";

constexpr char kBagRolloverTimeParameter[] = "bag_rollover_time";
constexpr char kMaxRecordTimeParameter[] = "max_record_time";
constexpr char kTopicsToRecordParameter[] = "topics_to_record";
constexpr char kWriteDirectoryParameter[] = "write_directory";
constexpr char kUploadTimeoutParameter[] = "upload_timeout";

constexpr uint32_t kBagRolloverTimeDefaultInSeconds = 30;
constexpr uint32_t kMaxRecordTimeDefaultInSeconds = 300;
constexpr char kWriteDirectoryDefault[] = "~/.ros/rr_rosbag_uploader/";
constexpr uint32_t kTimeOutDefaultInSeconds = 3600;

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

  // Set topics_to_record
  std::vector<std::string> topics_to_record;
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kTopicsToRecordParameter), topics_to_record)) {
    AWS_LOG_WARN(__func__, "Failed to load topics to record preference, defaulting to all topics.");
    topics_to_record.clear();
  }

  // Set write_directory
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kWriteDirectoryParameter), write_directory_input)) {
    write_directory_input = kWriteDirectoryDefault;
  }

  // Set operation time out in seconds
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kUploadTimeoutParameter), rolling_recorder_options.upload_timeout_s)) {
    rolling_recorder_options.upload_timeout_s = kTimeOutDefaultInSeconds;
  }

  if (Aws::Rosbag::Utils::ExpandAndCreateDir(write_directory_input, rolling_recorder_options.write_directory)) {
    AWS_LOG_INFO(__func__, "Starting rolling recorder node.");
    Aws::Rosbag::Utils::RosbagRecorder<Aws::Rosbag::Utils::Recorder> rosbag_recorder;
    Aws::Rosbag::RollingRecorder rolling_recorder;

    if (!rolling_recorder.InitializeRollingRecorder(rolling_recorder_options)) {
      AWS_LOG_INFO(__func__, "Failed to initialize rolling recorder. Shutting down.");
    } else {
      Aws::Rosbag::Utils::RecorderOptions options;
      options.split = true;
      options.max_duration = rolling_recorder_options.bag_rollover_time;
      options.record_all = false;
      if (topics_to_record.empty()) {
        options.record_all = true;
      } else {
        options.topics = std::move(topics_to_record);
      }
      options.prefix = rolling_recorder_options.write_directory;
      rosbag_recorder.Run(options, nullptr, [](int /*exit_code*/) { ros::shutdown(); });

      ros::MultiThreadedSpinner spinner(2);
      spinner.spin();

      AWS_LOG_INFO(__func__, "Finishing rolling recorder node.");
    }

    ros::shutdown();
  }

  Aws::Utils::Logging::ShutdownAWSLogging();

  return 0;
}
