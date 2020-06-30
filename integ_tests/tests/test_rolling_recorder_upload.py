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

    def tearDown(self):
        super(TestRollingRecorderUploadOnGoal, self).setUp()
        self.s3_client.delete_all_objects(self.s3_bucket_name)
        self.s3_client.delete_bucket(self.s3_bucket_name)

    def test_record_custom_topic(self):
        # Get the custom topic we specified in the test file
        self.topic_to_record = rospy.get_param("~topic_to_record")

        # Wait for rolling recorder node and action server to start
        self.wait_for_rolling_recorder_nodes()

        # Create publishers 
        self.test_publisher = rospy.Publisher(self.topic_to_record, String, queue_size=10)
        self.wait_for_rolling_recorder_node_to_subscribe_to_topic()
        
        # Find start time of active file
        active_rosbag = self.get_latest_bag_by_regex("*.bag.active")
        rospy.loginfo("Active rosbag: %s" % active_rosbag)
        active_rosbag_start_time = os.path.getctime(active_rosbag)
        start_time = rospy.Time.from_sec(active_rosbag_start_time - 1)

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

        latest_bag = self.get_latest_bag_by_regex("*.bag")

        # Send a goal to upload the bag data to S3
        # Create an Action client to send the goal
        self.action_client = actionlib.SimpleActionClient(ACTION, RollingRecorderAction)
        res = self.action_client.wait_for_server()
        self.assertTrue(res, 'Failed to connect to rolling recorder action server')

        # Create the goal and send through action client
        s3_folder = 'test_rr/'
        s3_subfolder = ''.join([random.choice(string.ascii_letters + string.digits) for _ in range(8)])  
        s3_destination = os.path.join(s3_folder, s3_subfolder)
        end_time = rospy.Time.now()
        goal = RollingRecorderGoal(destination=s3_destination)
        self.action_client.send_goal(goal)
        res = self.action_client.wait_for_result(rospy.Duration.from_sec(GOAL_COMPLETION_TIMEOUT_SEC))
        self.assertTrue(res, "Rolling Recorder Goal timed out")
        result = self.action_client.get_result()
        self.assertEquals(result.result.result, RESULT_SUCCESS)

        s3_key = os.path.join(s3_destination, os.path.basename(latest_bag))
        with tempfile.NamedTemporaryFile() as f:
            self.s3_client.download_file(self.s3_bucket_name, s3_key, f.name)
            bag = rosbag.Bag(f.name)
            total_bag_messages = 0
            for _, msg, _ in bag.read_messages():
                total_bag_messages += 1

            self.assertEquals(total_bag_messages, total_test_messages)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRollingRecorderUploadOnGoal, sys.argv)
