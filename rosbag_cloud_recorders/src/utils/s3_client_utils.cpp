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

#include <recorder_msgs/RollingRecorderAction.h>

constexpr uint32_t kTimeOutInSeconds = 30;
namespace Aws
{
namespace Rosbag
{
namespace Utils
{

file_uploader_msgs::UploadFilesGoal ConstructRosBagUploaderGoal(std::string destination, std::vector<std::string> & ros_bags_to_upload)
{
  AWS_LOG_INFO(__func__, "Constructing Uploader Goal.");
  file_uploader_msgs::UploadFilesGoal file_uploader_goal;
  file_uploader_goal.files = ros_bags_to_upload;
  file_uploader_goal.upload_location = destination;
  return file_uploader_goal;
}

RecorderErrorCode SendRosBagUploaderGoal(const file_uploader_msgs::UploadFilesGoal & goal,
  std::unique_ptr<T_simple_action_client> & rosbag_uploader_action_client,
  int & result_code)
{
  rosbag_uploader_action_client->sendGoal(goal);
  bool finished_before_timeout = rosbag_uploader_action_client->waitForResult(ros::Duration(kTimeOutInSeconds));
  if (!finished_before_timeout) {
    AWS_LOG_WARN(__func__, "Uploading rosbags to S3 did not finish before the time out.");
    return UPLOADING_TIMED_OUT;
  }

  AWS_LOG_INFO(__func__, "Uploading rosbags to S3 finished with an code: %d",rosbag_uploader_action_client->getResult()->result_code);
  result_code = rosbag_uploader_action_client->getResult()->result_code;
  return SUCCESS;
}

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws