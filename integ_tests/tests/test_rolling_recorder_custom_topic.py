#!/usr/bin/env python
# Copyright (c) 2020, Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

import os
import sys
import time

from std_msgs.msg import String

import rosbag
import rosnode
import rospy
import rostest
import rostopic

from rolling_recorder_test_base import RollingRecorderTestBase

PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'rolling_recorder_custom_topic'

class TestRollingRecorderCustomTopic(RollingRecorderTestBase):
    def test_record_custom_topic(self):
        # Get the custom topic we specified in the test file
        self.topic_to_record = rospy.get_param("~topic_to_record")

        # Wait for rolling recorder node to start
        self.wait_for_rolling_recorder_nodes()

        # Create publishers 
        self.test_publisher = rospy.Publisher(self.topic_to_record, String, queue_size=10)
        self.wait_for_rolling_recorder_node_to_subscribe_to_topic()

        # Find start time of active file
        active_rosbag = self.get_latest_bag_by_regex("*.bag.active")
        rospy.loginfo("Active rosbag: %s" % active_rosbag)
        active_rosbag_start_time = os.path.getctime(active_rosbag)

        # Calculate time active bag will roll over
        bag_finish_time = active_rosbag_start_time + self.bag_rollover_time
        bag_finish_time_remaining = bag_finish_time - time.time()
        rospy.loginfo("Bag finish time remaining: %f" % bag_finish_time_remaining)

        # Emit some data to the test topic
        total_test_messages = 10
        sleep_between_message = (bag_finish_time_remaining * 0.5)  / total_test_messages
        rospy.loginfo("Sleep between messages: %f" % sleep_between_message)
        for x in range(total_test_messages):
            self.test_publisher.publish("Test message %d" % x)
            time.sleep(sleep_between_message)

        # Wait for current bag to finish recording and roll over
        bag_finish_time_remaining = bag_finish_time - time.time()
        rospy.loginfo("Bag finish time remaining after publish: %f" % bag_finish_time_remaining)

        # Add 0.5s as it takes some time for bag rotation to occur
        time.sleep(bag_finish_time_remaining + 0.5) 
        
        # Check that the data is inside the latest rosbag
        latest_bag = self.get_latest_bag_by_regex("*.bag")
        rospy.loginfo("Latest bag: %s " % latest_bag)
        bag = rosbag.Bag(latest_bag)
        total_bag_messages = 0
        for _, msg, _ in bag.read_messages():
            total_bag_messages += 1

        self.assertEquals(total_bag_messages, total_test_messages)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRollingRecorderCustomTopic, sys.argv)
