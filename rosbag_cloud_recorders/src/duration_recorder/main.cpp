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
#include <cerrno>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>

#include <rosbag_cloud_recorders/duration_recorder/duration_recorder.h>
#include <rosbag_cloud_recorders/utils/file_utils.h>

namespace
{

constexpr char kNodeName[] = "rosbag_duration_recorder";

constexpr char kMinFreeSpaceParameter[] = "min_free_disk_space";
constexpr char kWriteDirectoryParameter[] = "write_directory";
constexpr char kUploadTimeoutParameter[] = "upload_timeout";
constexpr char kDeleteBagsAfterUploadParameter[] = "delete_bags_after_upload";

constexpr uint64_t kMinFreeSpaceDefaultInMebibytes = 1024;
constexpr char kWriteDirectoryDefault[] = "~/.ros/dr_rosbag_uploader/";
constexpr double kUploadTimeoutDefaultInSeconds = 3600.0;   // Default to 60 min timeout
constexpr bool kDeleteBagsAfterUploadDefault = false; // Default to not deleting bags after they have been uploaded

} // namespace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, kNodeName);
  Aws::Utils::Logging::InitializeAWSLogging(
        Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));

  Aws::Rosbag::DurationRecorderOptions duration_recorder_options;
  auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();

  int min_free_space;
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kMinFreeSpaceParameter), min_free_space)) {
    if (min_free_space < 0) {
      AWS_LOG_ERROR(__func__, "min_free_disk_space must be a positive integer.");
      return EXIT_FAILURE;
    }
    duration_recorder_options.min_free_space_mib = min_free_space;
  } else {
    duration_recorder_options.min_free_space_mib = kMinFreeSpaceDefaultInMebibytes;
  }
  std::string write_dir_input;
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kWriteDirectoryParameter), write_dir_input)) {
    write_dir_input = kWriteDirectoryDefault;
  }
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kUploadTimeoutParameter), duration_recorder_options.upload_timeout_s)) {
    duration_recorder_options.upload_timeout_s = kUploadTimeoutDefaultInSeconds;
  }
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader->ReadParam(Aws::Client::ParameterPath(kDeleteBagsAfterUploadParameter), duration_recorder_options.delete_bags_after_upload)) {
    duration_recorder_options.delete_bags_after_upload = kDeleteBagsAfterUploadDefault;
  }

  std::string write_dir;
  if (Aws::Rosbag::Utils::ExpandAndCreateDir(write_dir_input, write_dir)) {
    duration_recorder_options.write_directory = write_dir;
    AWS_LOG_INFO(__func__, "Starting duration recorder");

    Aws::Rosbag::DurationRecorder duration_recorder(duration_recorder_options);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
  } else {
    AWS_LOG_ERROR(__func__, "Failed to access rosbag write directory. Shutting down.");
    return EXIT_FAILURE;
  }

  Aws::Utils::Logging::ShutdownAWSLogging();

  return EXIT_SUCCESS;
}
