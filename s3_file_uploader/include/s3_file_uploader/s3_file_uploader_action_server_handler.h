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

#include <array>
#include <limits>
#include <vector>

#include <actionlib/server/action_server.h>
#include <file_uploader_msgs/UploadFilesAction.h>
#include <s3_common/s3_upload_manager.h>


#include<s3_file_uploader/s3_file_uploader_action_server_handler.h>

#include <aws/s3/S3Client.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>


#include <s3_common/utils.h>

namespace Aws{
namespace S3 {
  
using UploadFilesActionServer = actionlib::ActionServer<file_uploader_msgs::UploadFilesAction>;

template<typename T>
class S3FileUploaderActionServerHandler
{ 
public:
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
  
    auto outcome = upload_manager.UploadFiles(
      uploads, bucket, feedback_callback);
    file_uploader_msgs::UploadFilesResult result;
    if (outcome.IsSuccess()) {
      result.result_code.success = static_cast<uint8_t>(true);
      result.result_code.error_code = std::numeric_limits<int16_t>::lowest();
    } else {
      result.result_code.success = static_cast<uint8_t>(false);
      result.result_code.error_code = static_cast<int16_t>(outcome.GetError().GetErrorType());
    }
    for (auto const& upload : completed_uploads) {
      result.files_uploaded.push_back(upload.object_key);
    }
    if (actionlib_msgs::GoalStatus::PREEMPTING == goal_handle.getGoalStatus().status) {
      // Goal cancel has been requested
      goal_handle.setCanceled(result, "");
      return;
    }
    if (!outcome.IsSuccess()) {
      std::stringstream ss;
      ss << "Goal was aborted due to error uploading files. Error Message: " << outcome.GetError().GetMessage();
      goal_handle.setAborted(result, ss.str());
    } else {
      goal_handle.setSucceeded(result, "");
    }
  }

  static void CancelUploadToS3(S3UploadManager& upload_manager)
  {
    AWS_LOG_INFO(__func__, "Cancelling Goal");
    upload_manager.CancelUpload();
  }
};

}  // namespace S3
}  // namespace Aws