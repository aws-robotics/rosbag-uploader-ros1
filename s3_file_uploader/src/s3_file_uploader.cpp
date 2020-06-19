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
#include <array>
#include <vector>

#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <file_uploader_msgs/UploadFilesAction.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <s3_common/s3_upload_manager.h>
#include <s3_common/utils.h>

#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <ros/ros.h>

#include <s3_file_uploader/s3_file_uploader.h>
#include <s3_file_uploader/s3_file_uploader_action_server_handler.h>


constexpr char kBucketNameParameter[] = "s3_bucket";
constexpr char kEnableEncryptionParameter[] = "enable_encryption";
constexpr char kSpinnerThreadCountOverrideParameter[] = "spinner_thread_count";

/**
 * By default we use two threads to handle upload goals. You may specify a different setting
 * via the "spinner_thread_count" parameter.
 */
constexpr uint32_t kDefaultNumberOfSpinnerThreads = 2;

namespace Aws
{
namespace S3
{


S3FileUploader::S3FileUploader(std::unique_ptr<S3UploadManager> upload_manager) :
  node_handle_("~"),
  action_server_(node_handle_, "UploadFiles", false)
{
  parameter_reader_ = std::make_shared<Aws::Client::Ros1NodeParameterReader>();

  if (upload_manager) {
    upload_manager_ = move(upload_manager);
  } else {
    bool enable_encryption = false;
    if (Aws::AwsError::AWS_ERR_OK != parameter_reader_->ReadParam(Aws::Client::ParameterPath(kEnableEncryptionParameter), enable_encryption)) {
      AWS_LOG_INFO(__func__, "No user setting for data encryption provided, defaulting to no data encryption");
    }

    Aws::Client::ClientConfigurationProvider configuration_provider(parameter_reader_);
    Aws::Client::ClientConfiguration aws_sdk_config = configuration_provider.GetClientConfiguration();

    upload_manager_ = std::make_unique<S3UploadManager>(enable_encryption, aws_sdk_config);
  }
  
  action_server_.registerGoalCallback(
    [this](UploadFilesActionServer::GoalHandle goal_handle) {
      S3FileUploaderActionServerHandler<UploadFilesActionServer::GoalHandle>::UploadToS3(*upload_manager_, bucket_, goal_handle);
    }
  );
  
  action_server_.registerCancelCallback(
    [this](UploadFilesActionServer::GoalHandle /*goal_handle*/) {
      S3FileUploaderActionServerHandler<UploadFilesActionServer::GoalHandle>::CancelUploadToS3(*upload_manager_);
    }
  );
  
  action_server_.start();
}

void S3FileUploader::Spin()
{
  if (Aws::AwsError::AWS_ERR_OK != parameter_reader_->ReadParam(Aws::Client::ParameterPath(kBucketNameParameter), bucket_)) {
    AWS_LOG_ERROR(__func__, "Failed to load s3 bucket name, aborting. Check the configuration file for parameter s3_bucket");
    return;
  }

  uint32_t spinner_thread_count = kDefaultNumberOfSpinnerThreads;
  int spinner_thread_count_input;
  if (Aws::AwsError::AWS_ERR_OK ==
    parameter_reader_->ReadParam(Aws::Client::ParameterPath(kSpinnerThreadCountOverrideParameter),
                                 spinner_thread_count_input)) {
    if (spinner_thread_count_input < 0) {
      spinner_thread_count = 0;
    } else {
      spinner_thread_count = static_cast<uint32_t>(spinner_thread_count_input);
    }
  }

  AWS_LOG_INFO(__func__, "Starting S3FileUploader spinner with bucket %s and thread count %d\n", bucket_.c_str(), spinner_thread_count);
  ros::MultiThreadedSpinner executor(spinner_thread_count);
  executor.spin();
}

}  // namespace S3
}  // namespace Aws
