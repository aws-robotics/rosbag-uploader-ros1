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

import math
import os
import random
import string
import sys
import tempfile
import time

import actionlib
import rosbag
import rosnode
import rospy
import rostest
import rostopic
from s3_client import S3Client

from recorder_msgs.msg import RollingRecorderAction, RollingRecorderGoal
from rolling_recorder_test_base import RollingRecorderTestBase
from std_msgs.msg import String

PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'rolling_recorder_custom_topic'
ACTION = '/rolling_recorder/RosbagRollingRecord'
RESULT_SUCCESS = 0
GOAL_COMPLETION_TIMEOUT_SEC = 30.0

class TestRollingRecorderUploadOnGoal(RollingRecorderTestBase):
    def setUp(self):
        super(TestRollingRecorderUploadOnGoal, self).setUp()
        self.s3_region = rospy.get_param('/s3_file_uploader/aws_client_configuration/region')
        self.s3_bucket_name = rospy.get_param('/s3_file_uploader/s3_bucket')
        self.s3_client = S3Client(self.s3_region)
        self.s3_client.create_bucket(self.s3_bucket_name)
        self.s3_client.wait_for_bucket_create(self.s3_bucket_name)
        # Wait for rolling recorder node and action server to start
        self.wait_for_rolling_recorder_nodes()
        # Create publishers 
        self.topic_to_record = rospy.get_param('~topic_to_record')
        self.test_publisher = rospy.Publisher(self.topic_to_record, String, queue_size=None)
        self.wait_for_rolling_recorder_node_to_subscribe_to_topic()

    def tearDown(self):
        super(TestRollingRecorderUploadOnGoal, self).tearDown()
        self.s3_client.delete_all_objects(self.s3_bucket_name)
        self.s3_client.delete_bucket(self.s3_bucket_name)

    def test_record_upload(self):
        self.total_test_messages = 10

        (bag_name, expected_test_messages, s3_destination) = self.run_rolling_recorder()
        self.send_rolling_recorder_upload_goal(bag_name, expected_test_messages, s3_destination)

    def test_record_upload_multiple_times(self):
        self.total_test_messages = 10
        total_record_upload_attempts = 10

        for _ in range(total_record_upload_attempts):
            (bag_name, expected_test_messages, s3_destination) = self.run_rolling_recorder()
            self.send_rolling_recorder_upload_goal(bag_name, expected_test_messages, s3_destination)

    def _assert_bag_has_expected_messages(self, bag_file_path, expected_test_messages):
        bag = rosbag.Bag(bag_file_path)
        total_bag_messages = 0
        for _, msg, _ in bag.read_messages():
            total_bag_messages += 1
        self.assertGreaterEqual(total_bag_messages, expected_test_messages)

    def run_rolling_recorder(self):
        while True:
            # Find start time of the currently active rosbag
            (active_rosbag, active_rosbag_start_time) = self.get_latest_bag_by_regex('*.bag.active')
            rospy.loginfo('Active rosbag: %s' % active_rosbag)
            if active_rosbag == None:
                # this means in the middle of bag rotation, give a bit of time for bag rotation to occur
                time.sleep(0.01 * self.bag_rollover_time)
                continue

            # Calculate time active bag will roll over
            bag_finish_time = active_rosbag_start_time + self.bag_rollover_time
            bag_finish_time_remaining = bag_finish_time - time.time()
            rospy.loginfo('Bag finish time remaining: %f' % bag_finish_time_remaining)

            if bag_finish_time_remaining >= (0.7 * self.bag_rollover_time) or bag_finish_time_remaining > 2.0:
                # there is probably enough time remaining in the current active rosbag for the recorder to
                # receive and record all the messages that we will be sending to it
                break
            elif bag_finish_time_remaining > 0.0:
                # wait for a fresh bag if there's less than half of rollover time remaining for the current bag
                time.sleep(bag_finish_time_remaining)

        # Emit some data to the test topic
        max_test_messages_per_bag = 10
        num_test_messages = 0
        for x in range(max_test_messages_per_bag):
            self.test_publisher.publish('Test message %d' % x)
            if time.time() < bag_finish_time:
                num_test_messages += 1
            else:
                break
        rospy.loginfo('Number of messages sent to bag recorder: %f' % num_test_messages)

        # Wait for current bag to finish recording and roll over,
        # so that the bag that received the test messages is ready for upload
        bag_finish_time_remaining = bag_finish_time - time.time()
        rospy.loginfo('Bag finish time remaining after publish: %f' % bag_finish_time_remaining)
        time.sleep(bag_finish_time_remaining)
        while True:
            (latest_bag, _) = self.get_latest_bag_by_regex('*.bag')
            if latest_bag == active_rosbag[:-len('.active')]:
                break
            time.sleep(0.05 * self.bag_rollover_time)
        self._assert_bag_has_expected_messages(latest_bag, num_test_messages)

        # Send a goal to upload the bag data to S3
        # Create an Action client to send the goal
        self.action_client = actionlib.SimpleActionClient(ACTION, RollingRecorderAction)
        res = self.action_client.wait_for_server()
        self.assertTrue(res, 'Failed to connect to rolling recorder action server')

        # Create the goal and send through action client
        s3_folder = 'test_rr/'
        s3_subfolder = ''.join([random.choice(string.ascii_letters + string.digits) for _ in range(8)])  
        s3_destination = os.path.join(s3_folder, s3_subfolder)

        return (latest_bag, num_test_messages, s3_destination)

    def send_rolling_recorder_upload_goal(self, bag_name, expected_test_messages, s3_destination):
        end_time = rospy.Time.now()
        goal = RollingRecorderGoal(destination=s3_destination)
        self.action_client.send_goal(goal)
        res = self.action_client.wait_for_result(rospy.Duration.from_sec(GOAL_COMPLETION_TIMEOUT_SEC))
        self.assertTrue(res, 'Rolling Recorder Goal timed out')
        result = self.action_client.get_result()
        self.assertEquals(result.result.result, RESULT_SUCCESS)

        s3_key = os.path.join(s3_destination, os.path.basename(bag_name))
        with tempfile.NamedTemporaryFile() as f:
            rospy.loginfo('Downloading "%s" from S3' % s3_key)
            self.s3_client.download_file(self.s3_bucket_name, s3_key, f.name)
            self._assert_bag_has_expected_messages(f.name, expected_test_messages)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRollingRecorderUploadOnGoal, sys.argv)
