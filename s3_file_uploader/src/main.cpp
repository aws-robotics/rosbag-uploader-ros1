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

#include <aws/core/Aws.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <ros/ros.h>
#include <s3_common/s3_upload_manager.h>
#include <s3_file_uploader/s3_file_uploader.h>

using Aws::S3::S3FileUploader;

constexpr char kNodeName[] = "s3_file_uploader";

int main(int argc, char* argv[])
{
    Aws::Utils::Logging::InitializeAWSLogging(
        Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));
    Aws::SDKOptions options;
    Aws::InitAPI(options);

    ros::init(argc, argv, kNodeName);

    AWS_LOGSTREAM_INFO(__func__, "Starting S3FileUploader node");

    S3FileUploader file_uploader;
    file_uploader.Spin();

    Aws::ShutdownAPI(options);
    Aws::Utils::Logging::ShutdownAWSLogging();
    return 0;
}
