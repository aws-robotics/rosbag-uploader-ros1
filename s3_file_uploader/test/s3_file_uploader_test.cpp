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
#include <s3_common/s3_upload_manager.h>

using namespace Aws::S3;

using UploadFilesActionClient = actionlib::ActionClient<file_uploader_msgs::UploadFilesAction>;

using ::testing::Return;
using ::testing::_;

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
    ros::AsyncSpinner executor;
    ros::NodeHandle nh;
    UploadFilesActionClient action_client;
    std::unique_ptr<MockS3UploadManager> upload_manager;

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
};

TEST_F(S3UploaderTest, TestActionSucceeds)
{
    bool message_received = false;
    EXPECT_CALL(*upload_manager, IsAvailable())
        .WillRepeatedly(Return(true));
    EXPECT_CALL(*upload_manager, UploadFiles(_,_,_))
        .WillOnce(Return(S3ErrorCode::SUCCESS));

    S3FileUploader file_uploader(std::move(upload_manager));
    // Wait 10 seconds for server to start
    bool client_connected = action_client.waitForActionServerToStart(ros::Duration(10, 0));
    ASSERT_TRUE(client_connected);

    auto transition_call_back = [&](UploadFilesActionClient::GoalHandle gh){
        if (gh.getCommState() == actionlib::CommState::StateEnum::DONE){
            EXPECT_EQ(actionlib::TerminalState::StateEnum::SUCCEEDED, gh.getTerminalState().state_);
            message_received = true;
        }
    };
    file_uploader_msgs::UploadFilesGoal goal;

    auto gh = action_client.sendGoal(goal, transition_call_back);
    ros::Duration(1,0).sleep();
    ASSERT_TRUE(message_received);
    
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
