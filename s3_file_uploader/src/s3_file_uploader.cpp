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
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/s3/S3Client.h>

#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <ros/ros.h>

#include <file_uploader_msgs/UploadFilesAction.h>
#include <s3_common/s3_facade.h>

#include <s3_file_uploader/s3_file_uploader.h>

namespace Aws
{
namespace S3
{

S3FileUploader::S3FileUploader(std::unique_ptr<Aws::S3::S3Facade> s3_facade) :
    node_handle_("~"),
    action_server_(node_handle_, "UploadFiles", false),
    s3_facade_(std::move(s3_facade))
{
    action_server_.registerGoalCallback(
        boost::bind(&S3FileUploader::GoalCallBack, this, _1));
    action_server_.registerCancelCallback(
        boost::bind(&S3FileUploader::CancelGoalCallBack, this, _1));
    action_server_.start();
}

void S3FileUploader::GoalCallBack(UploadFilesActionServer::GoalHandle goal_handle)
{
    goal_handle.setAccepted();
    auto goal = goal_handle.getGoal();
    for (const auto& file_name: goal->files) {
        //bucket comes from config
        AWS_LOG_INFO(__func__,"Uploading file %s to %s", file_name.c_str(), goal->upload_location.c_str());
        s3_facade_->PutObject(file_name, "bucket", goal->upload_location);
    }
    file_uploader_msgs::UploadFilesResult result;
    goal_handle.setSucceeded(result, "");
}

void S3FileUploader::CancelGoalCallBack(UploadFilesActionServer::GoalHandle goal_handle)
{
    (void) goal_handle;
}

}  // namespace S3
}  // namespace Aws
