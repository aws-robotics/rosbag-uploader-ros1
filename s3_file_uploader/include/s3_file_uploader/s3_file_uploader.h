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

#pragma once

#include <actionlib/server/action_server.h>
#include <ros/ros.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <file_uploader_msgs/UploadFilesAction.h>
#include <s3_common/s3_upload_manager.h>

namespace Aws {
namespace S3 {

using UploadFilesActionServer = actionlib::ActionServer<file_uploader_msgs::UploadFilesAction>;

/**
 * S3FileUploader is a node that responds to actions to upload files to s3
 */
class S3FileUploader
{
public:
  explicit S3FileUploader(std::unique_ptr<S3UploadManager> upload_manager = nullptr);
  ~S3FileUploader() = default;
  void Spin();

private:
  ros::NodeHandle node_handle_;
  UploadFilesActionServer action_server_;
  std::unique_ptr<S3UploadManager> upload_manager_;
  std::shared_ptr<Aws::Client::Ros1NodeParameterReader> parameter_reader_;
  std::string bucket_;
};

}  // namespace S3
}  // namespace Aws
