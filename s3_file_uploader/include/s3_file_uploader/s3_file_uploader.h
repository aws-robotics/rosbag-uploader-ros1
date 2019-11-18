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
#include <ros/spinner.h>


#include <file_uploader_msgs/UploadFilesAction.h>
#include <s3_common/s3_facade.h>

namespace Aws
{
namespace S3
{

typedef actionlib::ActionServer<file_uploader_msgs::UploadFilesAction> UploadFilesActionServer;

/**
 * S3FileUploader is a node that responds to actions to upload files to s3
 */
class S3FileUploader
{
private:
    std::unique_ptr<UploadFilesActionServer> action_server_;
    ros::NodeHandle node_handle_;
    std::unique_ptr<Aws::S3::S3Facade> s3_facade_;

    void GoalCallBack(UploadFilesActionServer::GoalHandle goal);
    void CancelGoalCallBack(UploadFilesActionServer::GoalHandle goal);

public:
    S3FileUploader(std::unique_ptr<Aws::S3::S3Facade> facade);
    ~S3FileUploader() = default;
};

}  // namespace S3
}  // namespace Aws
