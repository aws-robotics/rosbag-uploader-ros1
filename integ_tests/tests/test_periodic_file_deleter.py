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


import glob
import os
import sys
import time
import unittest

from std_msgs.msg import String

import rosbag
import rosnode
import rospy
import rostest
import rostopic

from rolling_recorder_test_base import RollingRecorderTestBase

PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'periodic_file_deleter'

class TestPeriodicFileDeleter(RollingRecorderTestBase):
    def setUp(self):
        super(TestPeriodicFileDeleter, self).setUp()
        self.max_record_time = rospy.get_param("~max_record_time")
        self.periodic_deleter_interval = self.bag_rollover_time

    def test_record_custom_topic(self):
        # Wait for rolling recorder node to start
        self.wait_for_rolling_recorder_nodes()

        # Create publishers 
        self.test_publisher = rospy.Publisher('/some_topic', String, queue_size=10)
        self.wait_for_rolling_recorder_node_to_subscribe_to_topic()

        # Find start time of active file
        active_rosbag = self.get_latest_bag_by_regex("*.bag.active")
        rospy.loginfo("Active rosbag: %s" % active_rosbag)
        active_rosbag_start_time = os.path.getctime(active_rosbag)

        # Calculate time active bag will be deleted
        bag_finish_time = active_rosbag_start_time + self.periodic_deleter_interval
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
        # Add 0.3s as it takes some time for bag rotation to occur
        time.sleep(bag_finish_time_remaining + 0.3) 
        latest_bag = self.get_latest_bag_by_regex("*.bag")
        rospy.loginfo("Latest rosbag: %s" % latest_bag)

        # Check that the bag is deleted after at least one interval and max record time has passed
        time.sleep(self.periodic_deleter_interval + self.max_record_time)
        rospy.loginfo("Checking latest rosbag still exists: %s" % latest_bag)
        self.assertFalse(os.path.exists(latest_bag))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPeriodicFileDeleter, sys.argv)
