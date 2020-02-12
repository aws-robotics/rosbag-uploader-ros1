/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <actionlib/client/simple_action_client.h>

#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws/core/utils/logging/LogMacros.h>

#include <file_uploader_msgs/UploadFilesAction.h>

#include <recorder_msgs/RollingRecorderAction.h>

#include <rosbag_cloud_recorders/utils/s3_client_utils.h>
#include <rosbag_cloud_recorders/recorder_common_error_codes.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

file_uploader_msgs::UploadFilesGoal ConstructRosBagUploaderGoal(std::string destination,
  std::vector<std::string> & ros_bags_to_upload)
{
  AWS_LOG_INFO(__func__, "Constructing Uploader Goal.");
  file_uploader_msgs::UploadFilesGoal file_uploader_goal;
  file_uploader_goal.files = ros_bags_to_upload;
  file_uploader_goal.upload_location = std::move(destination);
  return file_uploader_goal;
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws
