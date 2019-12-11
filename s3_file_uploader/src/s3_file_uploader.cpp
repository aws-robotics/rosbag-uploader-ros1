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


namespace Aws
{
namespace S3
{


S3FileUploader::S3FileUploader(std::unique_ptr<S3UploadManager> upload_manager) :
    node_handle_("~"),
    action_server_(node_handle_, "UploadFiles", false),
    upload_manager_(std::move(upload_manager))
{
    parameter_reader_ = std::make_shared<Ros1NodeParameterReader>();

    if (upload_manager) {
        upload_manager_ = move(upload_manager);
    } else {
        ClientConfigurationProvider configuration_provider(parameter_reader_);
        ClientConfiguration aws_sdk_config = configuration_provider.GetClientConfiguration();
        upload_manager_ = std::make_unique<S3UploadManager>(aws_sdk_config);
    }

    action_server_.registerGoalCallback(
        boost::bind(&S3FileUploader::GoalCallBack, this, _1));
    action_server_.registerCancelCallback(
        boost::bind(&S3FileUploader::CancelGoalCallBack, this, _1));
    action_server_.start();
}

void S3FileUploader::GoalCallBack(UploadFilesActionServer::GoalHandle goal_handle)
{
    if (!upload_manager_->IsAvailable()) {
        goal_handle.setRejected();
        return;
    }
    goal_handle.setAccepted();
    auto goal = goal_handle.getGoal();
    std::vector<UploadDescription> uploads(goal->files.size());
    for (size_t i=0; i<goal->files.size(); i++) {
        uploads.at(i) = {
            goal->files[i],
            GenerateObjectKey(goal->files[i], goal->upload_location)
        };
    }
    std::vector<UploadDescription> completed_uploads;

    auto feedback_callback = [&](const std::vector<UploadDescription>& uploaded_files) {
        completed_uploads = uploaded_files;
        file_uploader_msgs::UploadFilesFeedback feedback;
        feedback.num_remaining = 0;
        feedback.num_uploaded = 0;
        goal_handle.publishFeedback(feedback);
    };

    // Bucket will be configurable in next PR
    auto result_code = upload_manager_->UploadFiles(
        uploads, bucket_, feedback_callback);
    file_uploader_msgs::UploadFilesResult result;
    result.code = result_code;
    for (auto const& upload : completed_uploads) {
        result.files_uploaded.push_back(upload.object_key);
    }
    goal_handle.setSucceeded(result, "");
}

void S3FileUploader::CancelGoalCallBack(UploadFilesActionServer::GoalHandle goal_handle)
{
    AWS_LOG_INFO(__func__, "Cancelling Goal");
    upload_manager_->CancelUpload();
    // Wait until cancel has finished
    while (!upload_manager_->IsAvailable()){
        ros::Duration(1.0).sleep();
    }
    goal_handle.setCanceled();
}

void S3FileUploader::Spin() {
    uint32_t spinner_thread_count = kDefaultNumberOfSpinnerThreads;
    int spinner_thread_count_input;
    if (Aws::AwsError::AWS_ERR_OK ==
        parameter_reader_->ReadParam(ParameterPath(kSpinnerThreadCountOverrideParameter),
                                     spinner_thread_count_input)) {
        spinner_thread_count = static_cast<uint32_t>(spinner_thread_count_input);
    }

    if (Aws::AwsError::AWS_ERR_OK !=
        parameter_reader_->ReadParam(ParameterPath(kBucketNameParameter), bucket_)) {
        AWS_LOG_ERROR(__func__, "Failed to load s3 bucket name, aborting. Check the configuration file for parameter s3_bucket");
        return;
    }
    AWS_LOG_INFO(__func__, "Starting S3FileUploader spinner with bucket %s and thread count %d\n", bucket_.c_str(), spinner_thread_count);

    ros::AsyncSpinner executor(spinner_thread_count);
    executor.start();
    ros::waitForShutdown();
    executor.stop();
}

}  // namespace S3
}  // namespace Aws
