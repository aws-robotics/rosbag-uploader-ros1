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
#include <actionlib/client/simple_action_client.h>

#include <file_uploader_msgs/UploadFilesAction.h>

namespace Aws
{
namespace Rosbag
{
namespace Utils
{

file_uploader_msgs::UploadFilesGoal ConstructRosBagUploaderGoal(std::string destination, std::vector<std::string> & ros_bags_to_upload);

RecorderErrorCode SendRosBagUploaderGoal(
  const file_uploader_msgs::UploadFilesGoal & goal,
  std::unique_ptr<T_simple_action_client> & rosbag_uploader_action_client,
  int & result_code);

}  // namespace Utils
}  // namespace Rosbag
}  // namespace Aws