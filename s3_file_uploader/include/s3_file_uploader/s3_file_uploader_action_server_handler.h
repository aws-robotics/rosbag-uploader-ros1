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
#include <file_uploader_msgs/UploadFilesAction.h>
#include <s3_common/s3_upload_manager.h>

#include <array>
#include <unordered_map>
#include <vector>

#include <file_uploader_msgs/UploadFilesAction.h>

#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>


#include <s3_common/utils.h>

namespace Aws{
namespace S3 {
    
using UploadFilesActionServer = actionlib::ActionServer<file_uploader_msgs::UploadFilesAction>;


const std::unordered_map<int, int> kS3ErrorCodeToActionResult (
{
  { S3ErrorCode::SUCCESS, file_uploader_msgs::UploadFilesResult::SUCCESS },
  { S3ErrorCode::CANCELLED, file_uploader_msgs::UploadFilesResult::UPLOAD_CANCELLED },
  { S3ErrorCode::UPLOADER_BUSY, file_uploader_msgs::UploadFilesResult::UPLOAD_FAILED_SERVICE_ERROR },
  { S3ErrorCode::S3_ACCESS_DENIED, file_uploader_msgs::UploadFilesResult::UPLOAD_FAILED_SERVICE_ERROR },
  { S3ErrorCode::FILE_COULDNT_BE_READ, file_uploader_msgs::UploadFilesResult::UPLOAD_FAILED_INVALID_INPUT },
  { S3ErrorCode::S3_NO_SUCH_BUCKET, file_uploader_msgs::UploadFilesResult::UPLOAD_FAILED_SERVICE_ERROR }
});

template<typename T>
class S3FileUploaderActionServerHandler
{ 
public:
    // Maps internal S3ErrorCodes to the error code for UploadFilesResult
    static  int GetResultCodeFromS3ErrorCode(const S3ErrorCode error_code)
    {
        auto search = kS3ErrorCodeToActionResult.find(error_code);
        if (search == kS3ErrorCodeToActionResult.end()) {
            return file_uploader_msgs::UploadFilesResult::UPLOAD_FAILED_SERVICE_ERROR;
        }
        return search->second;
    }
    static void UploadToS3(S3UploadManager& upload_manager, const std::string& bucket, T& goal_handle)
    {
        if (!upload_manager.IsAvailable()) {
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
            feedback.num_remaining = uploads.size() - uploaded_files.size();
            feedback.num_uploaded = uploaded_files.size();
            goal_handle.publishFeedback(feedback);
        };
    
        auto result_code = upload_manager.UploadFiles(
            uploads, bucket, feedback_callback);
        file_uploader_msgs::UploadFilesResult result;
        result.code = GetResultCodeFromS3ErrorCode(result_code);
        for (auto const& upload : completed_uploads) {
            result.files_uploaded.push_back(upload.object_key);
        }
        if (S3ErrorCode::SUCCESS != result_code) {
            goal_handle.setAborted(result, std::string("Goal was aborted due to error uploading files. S3ErrorCode: ") + std::to_string(result_code));
        } else {
            goal_handle.setSucceeded(result, "");
        }
    }

    static void CancelUploadToS3(S3UploadManager& upload_manager, T& goal_handle)
    {
        AWS_LOG_INFO(__func__, "Cancelling Goal");
        upload_manager.CancelUpload();
        // Wait until cancel has finished
        while (!upload_manager.IsAvailable()){
            ros::Duration(1.0).sleep();
        }
        goal_handle.setCanceled();
    }
};

}  // namespace S3
}  // namespace Aws