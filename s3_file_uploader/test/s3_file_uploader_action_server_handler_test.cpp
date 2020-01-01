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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <aws/core/Aws.h>

#include <file_uploader_msgs/UploadFilesAction.h>
#include <file_uploader_msgs/UploadFilesActionGoal.h>
#include <file_uploader_msgs/UploadFilesGoal.h>

#include <s3_file_uploader/s3_file_uploader_action_server_handler.h>
#include <s3_common/s3_upload_manager.h>

#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>

#include <ros/ros.h>

using namespace Aws::S3;

using ::testing::Return;
using ::testing::_;
using ::testing::ContainerEq;
using ::testing::Invoke;

class MockS3UploadManager : public S3UploadManager
{
public:
    MockS3UploadManager() = default;
    MOCK_METHOD3(UploadFiles, S3ErrorCode(const std::vector<UploadDescription> &,
        const std::string &,
        const boost::function<void (const std::vector<UploadDescription>&)>&));
    MOCK_METHOD0(CancelUpload, void());
    MOCK_CONST_METHOD0(IsAvailable,bool());
};

class MockGoalHandle 
{
public:
    MockGoalHandle() = default;
    MockGoalHandle(const MockGoalHandle& copy) {
        (void) copy;
    };
    MOCK_METHOD0(setRejected, void());
    MOCK_METHOD0(setAccepted, void());
    MOCK_METHOD0(setAborted, void());
    MOCK_METHOD0(setSucceeded, void());
    MOCK_METHOD0(setCanceled, void());
    MOCK_METHOD2(setAborted, void(const file_uploader_msgs::UploadFilesResult&, const std::string &));
    MOCK_METHOD2(setSucceeded, void(const file_uploader_msgs::UploadFilesResult&, const std::string &));

    MOCK_CONST_METHOD0(getGoal, boost::shared_ptr<file_uploader_msgs::UploadFilesGoal>());
    MOCK_CONST_METHOD1(publishFeedback, void(file_uploader_msgs::UploadFilesFeedback &));
};

class S3FileUploaderActionServerHandlerTests: public ::testing::Test 
{
protected:
    std::shared_ptr<MockS3UploadManager> upload_manager;
    std::shared_ptr<MockGoalHandle> goal_handle;
    boost::shared_ptr<file_uploader_msgs::UploadFilesGoal> goal;
public:
    S3FileUploaderActionServerHandlerTests():
        upload_manager(std::make_shared<MockS3UploadManager>()),
        goal_handle(std::make_shared<MockGoalHandle>()),
        goal(new file_uploader_msgs::UploadFilesGoal())
    {
        // do nothing
    }
    void givenUploadManagerAvailability(bool isAvailable) {
        EXPECT_CALL(*upload_manager, IsAvailable()).WillRepeatedly(Return(isAvailable));
    }
    
    void givenUnAvailableUploadManager() {
        givenUploadManagerAvailability(false);
    }
    
    void givenAvailableUploadManager() {
        givenUploadManagerAvailability(true);
    }
    
    void givenEventuallyAvailableUploadManager() {
        EXPECT_CALL(*upload_manager, IsAvailable()).Times(2).WillOnce(Return(false)).WillOnce(Return(true));
    }
    
    void givenUploadGoal() {
       goal->files.push_back("test_file_name");
       goal->upload_location = "my/upload/dir";
       EXPECT_CALL(*goal_handle, getGoal()).WillRepeatedly(Return(goal));
    }
    
    void givenUploadWithStatus(S3ErrorCode errorCode) {
        auto upload_files_action = [errorCode](
            const std::vector<UploadDescription> & upload_desc,
            const std::string & text,
            const boost::function<void (const std::vector<UploadDescription>&)>& feedback_fn) {
            (void) text;
            feedback_fn(upload_desc);
            return errorCode;
        };
        EXPECT_CALL(*upload_manager, UploadFiles(_, _, _)).WillOnce(Invoke(upload_files_action));
    }
    
    void givenSuccessfullUpload() {
        givenUploadWithStatus(S3ErrorCode::SUCCESS);
    }
    
    void givenFailedUpload() {
        givenUploadWithStatus(S3ErrorCode::S3_ACCESS_DENIED);
    }

    void assertGoalIsRejected() {
        EXPECT_CALL(*goal_handle, setRejected());
    }
    
    void assertGoalIsAccepted() {
        EXPECT_CALL(*goal_handle, setAccepted());
    }
    
    void asssertUploadIsCanceled() {
        EXPECT_CALL(*upload_manager, CancelUpload());
    }

    void assertGoalIsSuccess() {
        EXPECT_CALL(*goal_handle, setSucceeded(_, _));
    }
    
    void assertGoalIsCanceled() {
        EXPECT_CALL(*goal_handle, setCanceled());
    }
    
    void assertGoalIsAborted() {
        EXPECT_CALL(*goal_handle, setAborted(_, _));
    }
};

TEST_F(S3FileUploaderActionServerHandlerTests, TestInactiveUploadManager)
{
    givenUnAvailableUploadManager();
    assertGoalIsRejected();
    
    S3FileUploaderActionServerHandler<MockGoalHandle>::UploadToS3(*upload_manager, std::string("bucket_"), *goal_handle);
}

TEST_F(S3FileUploaderActionServerHandlerTests, TestUploadActionSucceeds)
{
    givenAvailableUploadManager();
    assertGoalIsAccepted();
    givenUploadGoal();
    givenSuccessfullUpload();
    assertGoalIsSuccess();
    
    S3FileUploaderActionServerHandler<MockGoalHandle>::UploadToS3(*upload_manager, std::string("bucket_"), *goal_handle);
}

TEST_F(S3FileUploaderActionServerHandlerTests, TestUploadActionFailure)
{
    givenAvailableUploadManager();
    assertGoalIsAccepted();
    givenUploadGoal();
    givenFailedUpload();
    assertGoalIsAborted();
    
    S3FileUploaderActionServerHandler<MockGoalHandle>::UploadToS3(*upload_manager, std::string("bucket_"), *goal_handle);
}

TEST_F(S3FileUploaderActionServerHandlerTests, TestCancelUploadToS3)
{
    givenEventuallyAvailableUploadManager();
    asssertUploadIsCanceled();
    assertGoalIsCanceled();
    
    S3FileUploaderActionServerHandler<MockGoalHandle>::CancelUploadToS3(*upload_manager, *goal_handle);
}

int main(int argc, char ** argv)
{
    Aws::SDKOptions options;
    Aws::InitAPI(options);
    ros::Time::init();
    ::testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    Aws::ShutdownAPI(options);
    return result;
}