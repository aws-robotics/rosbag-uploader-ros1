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

namespace
{

constexpr char kNodeName[] = "rosbag_rolling_recorder";

constexpr char kBagRolloverTimeParameter[] = "bag_rollover_time";
constexpr char kMaxRecordTimeParameter[] = "max_record_time";
constexpr char kMinFreeSpaceParameter[] = "min_free_disk_space";
constexpr char kTopicsToRecordParameter[] = "topics_to_record";
constexpr char kWriteDirectoryParameter[] = "write_directory";
constexpr char kUploadTimeoutParameter[] = "upload_timeout";

constexpr uint32_t kBagRolloverTimeDefaultInSeconds = 30;
constexpr uint32_t kMaxRecordTimeDefaultInSeconds = 300;
constexpr uint64_t kMinFreeSpaceDefaultInMebibytes = 1024;
constexpr char kWriteDirectoryDefault[] = "~/.ros/rr_rosbag_uploader/";
constexpr uint32_t kTimeOutDefaultInSeconds = 3600;

} // namespace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, kNodeName);
  Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));

  Aws::Rosbag::RollingRecorderOptions rolling_recorder_options;
  auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();

  // Set bag_rollover_time
  int bag_rollover_time_input;
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kBagRolloverTimeParameter), bag_rollover_time_input)) {
    rolling_recorder_options.bag_rollover_time = ros::Duration(bag_rollover_time_input);
  } else {
    rolling_recorder_options.bag_rollover_time = ros::Duration(kBagRolloverTimeDefaultInSeconds);
  }

  // Set max_record_time
  int max_record_time_input;
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kMaxRecordTimeParameter), max_record_time_input)) {
    rolling_recorder_options.max_record_time = ros::Duration(max_record_time_input);
  } else {
    rolling_recorder_options.max_record_time = ros::Duration(kMaxRecordTimeDefaultInSeconds);
  }

  // Set min_free_disk_space
  int min_free_space;
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kMinFreeSpaceParameter), min_free_space)) {
    if (min_free_space < 0) {
      AWS_LOG_ERROR(__func__, "min_free_disk_space must be a positive integer.");
      return EXIT_FAILURE;
    }
    rolling_recorder_options.min_free_space_mib = min_free_space;
  } else {
    rolling_recorder_options.min_free_space_mib = kMinFreeSpaceDefaultInMebibytes;
  }

  // Set topics_to_record
  std::vector<std::string> topics_to_record;
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kTopicsToRecordParameter), topics_to_record)) {
    AWS_LOG_INFO(__func__, "Failed to load topics to record preference, defaulting to all topics.");
    topics_to_record.clear();
  }

  // Set write_directory
  std::string write_directory_input;
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

    if (rolling_recorder.InitializeRollingRecorder(rolling_recorder_options)) {
      Aws::Rosbag::Utils::RecorderOptions options;
      options.split = true;
      options.max_duration = rolling_recorder_options.bag_rollover_time;
      options.min_space = 1024 * 1024 * rolling_recorder_options.min_free_space_mib; // mebibytes to bytes
      options.min_space_str = std::to_string(rolling_recorder_options.min_free_space_mib) + 'M';
      if (topics_to_record.empty()) {
        options.record_all = true;
      } else {
        options.record_all = false;
        options.topics = std::move(topics_to_record);
      }
      options.prefix = rolling_recorder_options.write_directory;
      rosbag_recorder.Run(options, nullptr, [](int /*exit_code*/) { ros::shutdown(); });

      ros::MultiThreadedSpinner spinner(2);
      spinner.spin();

      AWS_LOG_INFO(__func__, "Finishing rolling recorder node.");
    } else {
      AWS_LOG_ERROR(__func__, "Failed to initialize rolling recorder. Shutting down.");
      return EXIT_FAILURE;
    }

    ros::shutdown();
  } else {
    AWS_LOG_ERROR(__func__, "Failed to access rosbag write directory. Shutting down.");
    return EXIT_FAILURE;
  }

  Aws::Utils::Logging::ShutdownAWSLogging();

  return EXIT_SUCCESS;
}
