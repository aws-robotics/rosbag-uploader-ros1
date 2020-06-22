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
import unittest

import actionlib
import rospy
import rostest

from s3_client import S3Client

from file_helpers import get_latest_bag_by_regex, get_latest_bags_by_regex
from recorder_msgs.msg import DurationRecorderAction, DurationRecorderGoal

ACTION = '/duration_recorder/RosbagDurationRecord'
TEST_NODE_NAME = 'duration_record_action_client'
AWS_DEFAULT_REGION = 'us-west-2'


class DurationRecorderTestBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node(TEST_NODE_NAME, log_level=rospy.DEBUG)
        s3_region = rospy.get_param('/s3_file_uploader/aws_client_configuration/region')
        s3 = S3Client(s3_region)
        s3_bucket_name = rospy.get_param('/s3_file_uploader/s3_bucket')
        s3.create_bucket(s3_bucket_name)
        s3.wait_for_bucket_create(s3_bucket_name)

    @classmethod
    def tearDownClass(cls):
        s3_region = rospy.get_param('/s3_file_uploader/aws_client_configuration/region')
        s3 = S3Client(s3_region)
        s3_bucket_name = rospy.get_param('/s3_file_uploader/s3_bucket')
        s3.delete_all_objects(s3_bucket_name)
        s3.delete_bucket(s3_bucket_name)

    def setUp(self):
        self.action_client = self._create_duration_recorder_action_client()
        self.rosbag_directory = os.path.expanduser(rospy.get_param('~write_directory'))

    def tearDown(self):
        self.delete_all_rosbags()

    def _create_duration_recorder_action_client(self):
        action_client = actionlib.SimpleActionClient(ACTION, DurationRecorderAction)
        res = action_client.wait_for_server()
        self.assertTrue(res, 'Failed to connect to action server')
        return action_client

    def record_for_duration(self, duration=5, topics=None):
        if topics is None:
            topics = ['/rosout']
        goal = DurationRecorderGoal(
            duration=rospy.Duration.from_sec(duration),
            topics_to_record=topics
        )
        self.action_client.send_goal(goal)
        res = self.action_client.wait_for_result(rospy.Duration.from_sec(30.0))
        self.assertTrue(res, 'Timed out waiting for result after sending Duration Recorder Goal')
        return self.action_client.get_result()

    def check_rosbags_were_recorded(self, start_time, total_bags):
        latest_bags = self.get_latest_bags_by_regex("*.bag", total_bags)
        total_bags_found = len(latest_bags)
        self.assertEquals(total_bags_found, total_bags, "Expected %d bags but only found %d" % (total_bags, total_bags_found))
        for bag_path in latest_bags:
            bag_create_time = os.path.getctime(bag_path)
            self.assertGreater(bag_create_time, start_time)

    def delete_all_rosbags(self):
        all_bags = get_latest_bags_by_regex(self.rosbag_directory, "*.bag")
        for bag_path in all_bags:
            os.remove(bag_path)

    def get_latest_bag_by_regex(self, regex_pattern):
        return get_latest_bag_by_regex(self.rosbag_directory, regex_pattern)

    def get_latest_bags_by_regex(self, regex_pattern, count):
        return get_latest_bags_by_regex(self.rosbag_directory, regex_pattern, count)
