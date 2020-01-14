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

import filecmp
import glob
import os
import random
import string
import sys
import tempfile
import time
import unittest

import actionlib
from std_msgs.msg import String

import rosbag
import rosnode
import rospy
import rostest
import rostopic

PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'rolling_recorder'
ACTION = ''
TEST_NODE_NAME = 'test_rolling_recorder_action_client'
AWS_DEFAULT_REGION = 'us-west-2'


class TestRollingRecorder(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node(TEST_NODE_NAME, log_level=rospy.DEBUG)

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.rrnode_start_timeout = 10
        pass

    def tearDown(self):
        pass

    # The rolling recorder should record the topic /rosout by default 
    # TODO: This will need to be run from another launch file to not set the topics_to_record arg
    def test_record_default_topics(self):
        pass

    def test_record_custom_topic(self):
        topics_to_record = rospy.get_param("~topics_to_record")
        rospy.loginfo("Topics to record: %s" % topics_to_record)
        # Wait for rolling recorder node to start
        self.wait_for_rolling_recorder_nodes()

        # Create publishers 
        test_topic = topics_to_record.split(' ')[0]
        self.ft_publisher = rospy.Publisher(test_topic, String, queue_size=10)
        self.wait_for_rolling_recorder_node_to_subscribe_to_topic()

        # Emit some data for 10 seconds
        total_test_messages = 10
        for x in range(total_test_messages):
            self.ft_publisher.publish("Test message %d" % x)
            time.sleep(0.1)

        # Wait for bag to roll-over or finish recording it
        time.sleep(1.5)
        
        # Check for rosbag recording on disk
        rosbag_directory = rospy.get_param("~write_directory")
        rospy.loginfo("Rosbag directory: %s" % rosbag_directory)
        rosbag_files = glob.iglob(rosbag_directory + '/*.bag')
        rosbag_paths = [os.path.join(rosbag_directory, filename) for filename in rosbag_files]
        rosbags_sorted = sorted(rosbag_paths, key=os.path.getctime, reverse=True)
        rospy.loginfo("Sorted rosbags: %s " % rosbags_sorted)

        # Check that the data is inside the latest rosbag
        latest_bag = rosbags_sorted[0]
        rospy.loginfo("Latest bag: %s " % latest_bag)
        bag = rosbag.Bag(latest_bag)
        total_bag_messages = 0
        for _, msg, _ in bag.read_messages():
            rospy.loginfo("Msg: %s" % msg)
            total_bag_messages += 1

        self.assertEquals(total_bag_messages, total_test_messages)

    def wait_for_rolling_recorder_nodes(self):
        required_nodes = set([
            '/rolling_recorder',
            '/rosbag_record'
        ])
        while not required_nodes.issubset(rosnode.get_node_names()):
            time.sleep(0.1)

    def wait_for_rolling_recorder_node_to_subscribe_to_topic(self):
        rostopic.wait_for_subscriber(self.ft_publisher, self.rrnode_start_timeout)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRollingRecorder, sys.argv)
