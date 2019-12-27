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

#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <aws/core/Aws.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <s3_file_uploader/s3_file_uploader.h>
#include <file_uploader_msgs/UploadFilesAction.h>
#include <s3_common/s3_common_error_codes.h>
#include <s3_common/s3_upload_manager.h>
#include <s3_common/utils.h>

using namespace Aws::S3;

using UploadFilesActionClient = actionlib::ActionClient<file_uploader_msgs::UploadFilesAction>;

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

class S3UploaderTest : public ::testing::Test
{
protected:
    file_uploader_msgs::UploadFilesGoal goal;
    ros::AsyncSpinner executor;
    ros::NodeHandle nh;
    UploadFilesActionClient action_client;
    std::unique_ptr<MockS3UploadManager> upload_manager;
    UploadFilesActionClient::GoalHandle goal_handle;

    void TearDown() override
    {
        executor.stop();
    }
public:
    S3UploaderTest():
        executor(0), nh("~"), action_client(nh, "UploadFiles"),
        upload_manager(std::make_unique<MockS3UploadManager>())
    {
        executor.start();
    }


    void SanityGoalCheck(actionlib::TerminalState::StateEnum goal_terminal_state) {
        bool message_received = false;
        // Wait 10 seconds for server to start
        bool client_connected = action_client.waitForActionServerToStart(ros::Duration(10, 0));
        ASSERT_TRUE(client_connected);

        auto transition_call_back = [&](UploadFilesActionClient::GoalHandle gh){
            if (gh.getCommState() == actionlib::CommState::StateEnum::DONE){
                EXPECT_EQ(goal_terminal_state, gh.getTerminalState().state_);
                message_received = true;
            }
        };

        auto gh = action_client.sendGoal(goal, transition_call_back);
        ros::Duration(1, 0).sleep();
        ASSERT_TRUE(message_received);
        goal_handle = gh;
    }
};

TEST_F(S3UploaderTest, TestActionSucceeds)
{
    EXPECT_CALL(*upload_manager, IsAvailable())
        .WillRepeatedly(Return(true));
    // Dummy action that invokes the feedback callback.
    auto upload_files_action = [](
        const std::vector<UploadDescription> & upload_desc,
        const std::string & text,
        const boost::function<void (const std::vector<UploadDescription>&)>& feedback_fn) {
        (void) text;
        feedback_fn(upload_desc);
        return S3ErrorCode::SUCCESS;
    };
    EXPECT_CALL(*upload_manager, UploadFiles(_, _, _)).WillOnce(Invoke(upload_files_action));

    S3FileUploader file_uploader(std::move(upload_manager));
    goal.files.push_back("test_file_name");
    goal.upload_location = "/my/upload/dir";
    SanityGoalCheck(actionlib::TerminalState::StateEnum::SUCCEEDED);

    auto result = goal_handle.getResult();
    ASSERT_EQ("/my/upload/dir/test_file_name", result->files_uploaded.at(0));
    ASSERT_EQ(1, result->files_uploaded.size());
}

TEST_F(S3UploaderTest, TestActionSucceedsNoUploadManagerProvided)
{
    S3FileUploader file_uploader;
    SanityGoalCheck(actionlib::TerminalState::StateEnum::SUCCEEDED);
}

TEST_F(S3UploaderTest, TestGoalCancellation) {
    EXPECT_CALL(*upload_manager, IsAvailable())
        .WillOnce(Return(true)).WillOnce(Return(false)).WillRepeatedly(Return(true));
    EXPECT_CALL(*upload_manager, UploadFiles(_,_,_))
        .WillRepeatedly(Return(S3ErrorCode::SUCCESS));
    EXPECT_CALL(*upload_manager, CancelUpload())
        .Times(1);

    S3FileUploader file_uploader(std::move(upload_manager));

    goal.files.push_back("test_file_name");
    goal.upload_location = "/my/upload/dir";

    bool client_connected = action_client.waitForActionServerToStart(ros::Duration(10, 0));
    ASSERT_TRUE(client_connected);

    auto transition_call_back = [&](UploadFilesActionClient::GoalHandle gh) {
        (void) gh;
        ros::Duration(2, 0).sleep();
    };
    auto gh = action_client.sendGoal(goal, transition_call_back);
    gh.cancel();
}

TEST_F(S3UploaderTest, TestUploadManagerUnavailableGoalRejection)
{
    EXPECT_CALL(*upload_manager, IsAvailable())
        .WillRepeatedly(Return(false));
    EXPECT_CALL(*upload_manager, UploadFiles(_,_,_))
        .Times(0);
    S3FileUploader file_uploader(std::move(upload_manager));
    // Since the upload manager is unavailable, we expect the goal to be rejected
    SanityGoalCheck(actionlib::TerminalState::StateEnum::REJECTED);
}

TEST_F(S3UploaderTest, TestMultipleFilesActionSucceeds)
{
    size_t files_count = 1000;
    std::vector<UploadDescription> uploads(files_count);
    goal.upload_location = "/my/upload/dir";
    for (size_t i = 0; i < files_count; i++) {
        goal.files.push_back("test_file" + i);
        uploads.at(i) = {goal.files[i], GenerateObjectKey(goal.files[i], goal.upload_location)};
    }

    EXPECT_CALL(*upload_manager, IsAvailable())
        .WillRepeatedly(Return(true));
    EXPECT_CALL(*upload_manager, UploadFiles(ContainerEq(uploads),_,_))
        .WillOnce(Return(S3ErrorCode::SUCCESS));

    S3FileUploader file_uploader(std::move(upload_manager));
    SanityGoalCheck(actionlib::TerminalState::StateEnum::SUCCEEDED);
}

TEST_F(S3UploaderTest, TestSpin)
{
    EXPECT_CALL(*upload_manager, IsAvailable())
        .WillRepeatedly(Return(true));

    // No parameters defined - should just exit
    nh.deleteParam("spinner_thread_count");
    nh.deleteParam("s3_bucket");
    S3FileUploader file_uploader(std::move(upload_manager));
    file_uploader.Spin();

    // Spinner thread count set, but no s3 bucket
    nh.setParam("spinner_thread_count", 1);
    file_uploader.Spin();
}

int main(int argc, char** argv)
{
    Aws::SDKOptions options;
    Aws::InitAPI(options);
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node");
    auto result = RUN_ALL_TESTS();
    ros::shutdown();
    Aws::ShutdownAPI(options);
    return result;
}
