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

#include <ros/ros.h>
#include <s3_file_uploader/s3_file_uploader.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "s3_file_uploader");
    Aws::S3::S3FileUploader file_uploader;
    ros::spin();
    return 0;
}
