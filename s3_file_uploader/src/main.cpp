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

using namespace Aws::S3;

int main(int argc, char* argv[])
{
    Aws::SDKOptions options;
    Aws::InitAPI(options);

    char node_name[] = "s3_file_uploader";
    ros::init(argc, argv, node_name);    
    Aws::Utils::Logging::InitializeAWSLogging(
        Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(node_name));

    AWS_LOGSTREAM_INFO(__func__, "Starting S3FileUploader node");

    ros::AsyncSpinner executor(0);
    executor.start();

    auto upload_manager = std::make_unique<S3UploadManager>();
    S3FileUploader file_uploader(std::move(upload_manager));

    ros::waitForShutdown();
    executor.stop();
    Aws::ShutdownAPI(options);
    Aws::Utils::Logging::ShutdownAWSLogging();

    return 0;
}
