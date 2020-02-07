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

#include <boost/filesystem.hpp>

#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_common/fs_utils/wordexp_ros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>

#include <rosbag_cloud_recorders/duration_recorder/duration_recorder.h>

constexpr char kNodeName[] = "rosbag_duration_recorder";
constexpr char kWriteDirectoryParameter[] = "write_directory";

int main(int argc, char* argv[])
{
  ros::init(argc, argv, kNodeName);
  Aws::Utils::Logging::InitializeAWSLogging(
        Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));

  Aws::Rosbag::DurationRecorderOptions duration_recorder_options;

  std::string write_dir_input;
  duration_recorder_options.write_directory = "~/.ros/dr_rosbag_uploader/";

  auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();
  if (Aws::AwsError::AWS_ERR_OK == parameter_reader->ReadParam(Aws::Client::ParameterPath(kWriteDirectoryParameter), write_dir_input)) {
    wordexp_t wordexp_result;
    int result = wordexp_ros(write_dir_input.c_str(), &wordexp_result, 0);
    // Directory was successfully read and expanded
    if (0 == result && wordexp_result.we_wordc == 1) {
      duration_recorder_options.write_directory = *(wordexp_result.we_wordv);
    }
  }
  if (!boost::filesystem::exists(duration_recorder_options.write_directory)) {
    boost::filesystem::create_directories(duration_recorder_options.write_directory);
  }

  AWS_LOG_INFO(__func__, "Starting duration recorder");

  Aws::Rosbag::DurationRecorder duration_recorder(duration_recorder_options);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  Aws::Utils::Logging::ShutdownAWSLogging();
  
  return 0;
}
