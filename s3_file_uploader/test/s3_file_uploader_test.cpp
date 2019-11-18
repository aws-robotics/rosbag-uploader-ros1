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

#include <gtest/gtest.h>

#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <aws/core/Aws.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <file_uploader_msgs/UploadFilesAction.h>
#include <s3_file_uploader/s3_file_uploader.h>

typedef actionlib::ActionClient<file_uploader_msgs::UploadFilesAction> UploadFilesActionClient;

TEST(S3UploaderTest, TestActionReceived)
{
    ros::AsyncSpinner executor(0);
    executor.start();
    bool message_received = false;
    auto s3_client = std::make_unique<Aws::S3::S3Client>();
    auto s3_facade = std::make_unique<Aws::S3::S3Facade>(std::move(s3_client));
    Aws::S3::S3FileUploader file_uploader(std::move(s3_facade));
    ros::NodeHandle nh("~");
    auto action_client = new UploadFilesActionClient(nh, "UploadFiles");
    // Wait 10 seconds for server to start
    bool client_connected = action_client->waitForActionServerToStart(ros::Duration(10, 0));
    ASSERT_TRUE(client_connected);
    auto transition_call_back = [&message_received](UploadFilesActionClient::GoalHandle goal){
        EXPECT_EQ(goal.getTerminalState().state_, actionlib::TerminalState::StateEnum::SUCCEEDED);
        message_received = true;
    };
    file_uploader_msgs::UploadFilesGoal goal;
    auto gh = action_client->sendGoal(goal, transition_call_back);
    ros::Duration(1,0).sleep();
    ASSERT_TRUE(message_received);
    executor.stop();
}

int main(int argc, char** argv)
{
    // Should be removed once S3_Facade is mocked out
    Aws::SDKOptions options;
    Aws::InitAPI(options);
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node");
    auto result = RUN_ALL_TESTS();
    ros::shutdown();
    Aws::ShutdownAPI(options);
    return result;
}