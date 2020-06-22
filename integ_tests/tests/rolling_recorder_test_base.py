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
import unittest

import rosbag
import rosnode
import rospy
import rostest
import rostopic

from file_helpers import get_latest_bag_by_regex, get_latest_bags_by_regex
from std_msgs.msg import String

TEST_NODE_NAME = 'test_rolling_recorder_client'
ROLLING_RECORDER_NODE_START_TIMEOUT = 5

# Max time it should take for the current bag to go from active to inactive
# as bags are not immediately rolled switched because of polling delays
BAG_DEACTIVATE_TIME = 0.5

class RollingRecorderTestBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node(TEST_NODE_NAME, log_level=rospy.DEBUG)

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.bag_deactivate_time = BAG_DEACTIVATE_TIME
        self.bag_rollover_time = rospy.get_param("~bag_rollover_time")
        self.rosbag_directory = os.path.expanduser(rospy.get_param("~write_directory"))

    def tearDown(self):
        pass

    def wait_for_rolling_recorder_nodes(self, timeout=5):
        required_nodes = set(['/rolling_recorder'])
        start_time = time.time()
        while not required_nodes.issubset(rosnode.get_node_names()):
            if time.time() > start_time + timeout:
                raise Exception("Timed out waiting for rolling recorder nodes")
            time.sleep(0.1)

    def wait_for_rolling_recorder_node_to_subscribe_to_topic(self):
        rostopic.wait_for_subscriber(self.test_publisher, ROLLING_RECORDER_NODE_START_TIMEOUT)

    def get_latest_bag_by_regex(self, regex_pattern):
        return get_latest_bag_by_regex(self.rosbag_directory, regex_pattern)

    def get_latest_bags_by_regex(self, regex_pattern, count):
        return get_latest_bags_by_regex(self.rosbag_directory, regex_pattern, count)