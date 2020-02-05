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
import random
import string
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
NAME = 'rolling_recorder_default_topics'

class TestRollingRecorderDefaultTopics(RollingRecorderTestBase):
    # Curently by default the rolling_recorder records on all topics.
    # This test ensures that when posting to a random topic with default settings
    # the messages will be recorded correctly 
    def test_record_default_topics(self):
        # Wait for rolling recorder node to start
        self.wait_for_rolling_recorder_nodes()

        # Create publishers 
        topic_name = '/my_random_topic_' + ''.join([random.choice(string.ascii_letters + string.digits) for _ in range(8)])
        self.test_publisher = rospy.Publisher(topic_name, String, queue_size=10)
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

        # Wait for all messages to be saved to bags.
        # Because of buffering some messages may go into a second bag, which is
        # why we're waiting for it to roll over twice
        bag_finish_time_remaining = bag_finish_time - time.time()
        time.sleep(bag_finish_time_remaining + self.bag_rollover_time + self.bag_deactivate_time)
        
        # Check that the data is inside the latest rosbags
        latest_bags = self.get_latest_bags_by_regex("*.bag", 2)
        rospy.loginfo("Latest bags: %s " % latest_bags)
        total_messages = 0
        total_topic_messages = 0
        for bag_path in latest_bags:
            bag = rosbag.Bag(bag_path)
            for topic, msg, _ in bag.read_messages():
                total_messages += 1
                if topic == topic_name:
                    total_topic_messages += 1

        # Ensure that all messages published to this topic are recorded
        self.assertEquals(total_topic_messages, total_test_messages)
        # Ensure that more topics than just this topic are recorded
        # The additional messages will be logs about what the rosbag recorder is doing
        # published by the rosbag recorder itself, such as Opening/Closing rosbags
        # as well as any logs this system has written to /rosout. 
        self.assertTrue(total_messages > total_topic_messages)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRollingRecorderDefaultTopics, sys.argv)
