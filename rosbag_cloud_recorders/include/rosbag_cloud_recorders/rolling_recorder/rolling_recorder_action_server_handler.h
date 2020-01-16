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
#include <recorder_msgs/RollingRecorderAction.h>
#include <rosbag_cloud_recorders/rolling_recorder/rolling_recorder_action_server_handler.h>


namespace Aws{
namespace Rosbag{

using RollingRecorderActionServer = actionlib::ActionServer<recorder_msgs::RollingRecorderAction>;

template<typename T>
class RollingRecorderActionServerHandler
{
public:
    static void RollingRecorderRosbagUpload(T& goal_handle)
    {
        goal_handle.setRejected();
    }

    static void CancelRollingRecorderRosbagUpload(T& goal_handle)
    {
      goal_handle.setCanceled();
    }
};

}  // namespace Rosbag
}  // namespace Aws
