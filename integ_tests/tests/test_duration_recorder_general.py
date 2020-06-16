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

from functools import partial
import os
import random
import string
import sys
import threading
import tempfile
import time

import rosbag
import rospy
import rostest
from s3_client import S3Client

from std_msgs.msg import String
from recorder_msgs.msg import DurationRecorderResult, DurationRecorderGoal
from duration_recorder_test_base import DurationRecorderTestBase

PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'test_duration_record_general'
RESULT_CODE_SUCCESS = 0
RESULT_CODE_SKIPPED = 2

class TestDurationRecorderGeneral(DurationRecorderTestBase):
    def test_record_duration(self):
        start_time = time.time()
        action_result = self.record_for_duration(1)
        self.assertEquals(action_result.result.result, RESULT_CODE_SUCCESS)
        self.check_rosbags_were_recorded(start_time, 1)

    def test_record_multiple_times(self):
        start_time = time.time()
        total_recordings = 25
        for _ in range(total_recordings):
            action_result = self.record_for_duration(1)
            self.assertEquals(action_result.result.result, RESULT_CODE_SUCCESS)
        self.check_rosbags_were_recorded(start_time, total_recordings)

        
    def test_record_overlapping_times(self):
        """
        When attempting to record while it's already recording, ensure it 
        rejects the request gracefully
        """
        primary_goal_action_client = self._create_duration_recorder_action_client()
        start_time = time.time()
        record_time = 5
        total_attempted_recordings = 25
        time_between_attempts = (record_time / total_attempted_recordings) / 2

        # Start the duration recorder for `record_time` seconds
        goal = DurationRecorderGoal(
            duration=rospy.Duration.from_sec(record_time),
            topics_to_record=['/rosout']
        )
        primary_goal_action_client.send_goal(goal)

        # Try to record multiple times while it's already recording using default action client
        for _ in range(total_attempted_recordings):
            action_result = self.record_for_duration(1)
            self.assertEquals(action_result.result.result, RESULT_CODE_SKIPPED)
            time.sleep(time_between_attempts)

        res = primary_goal_action_client.wait_for_result(rospy.Duration.from_sec(15.0))
        self.assertTrue(res, 'Timed out waiting for result after sending Duration Recorder Goal')

        # Check only one bag was recorded since the start
        self.check_rosbags_were_recorded(start_time, 1)

    def test_record_specific_topic(self):
        start_time = time.time()
        topic_name = '/my_random_topic_' + create_random_word(8) 
        duration = 5
        interval = 0.1
        total_test_messages = 10

        # Publish some data to the topic before recording is not started.
        # This data SHOULD NOT be recorded into the rosbag
        self.publish_periodic_data_to_topic(topic_name, interval, total_test_messages)

        # Start the duration recorder for `duration` seconds
        goal = DurationRecorderGoal(
            duration=rospy.Duration.from_sec(duration),
            topics_to_record=[topic_name]
        )
        self.action_client.send_goal(goal)

        # Wait for duration recorder to start recording
        time.sleep(0.5)

        # Publish some data to that topic
        self.publish_periodic_data_to_topic(topic_name, interval, total_test_messages)

        # Wait for the duration recorder to finish
        self.action_client.wait_for_result(rospy.Duration.from_sec(10.0))
        action_result = self.action_client.get_result()

        # Publish some data to the topic after recording has finished
        # This data SHOULD NOT be recorded into the rosbag
        self.publish_periodic_data_to_topic(topic_name, interval, total_test_messages)

        # Ensure the duration recorder created the bag correctly
        self.assertEquals(action_result.result.result, RESULT_CODE_SUCCESS)
        self.check_rosbags_were_recorded(start_time, 1)

        # Ensure that the rosbag contains all the test messages
        latest_bag = self.get_latest_bag_by_regex("*.bag")
        total_topic_messages = 0
        bag = rosbag.Bag(latest_bag)
        for topic, msg, _ in bag.read_messages():
            if topic == topic_name:
                total_topic_messages += 1
        self.assertEquals(total_topic_messages, total_test_messages)

        # Ensure that the rosbag uploaded to S3 contains all the test messages
        s3_region = rospy.get_param('/s3_file_uploader/aws_client_configuration/region')
        s3_client = S3Client(s3_region)
        s3_bucket_name = rospy.get_param('/s3_file_uploader/s3_bucket')
        s3_key = os.path.basename(latest_bag)
        with tempfile.NamedTemporaryFile() as f:
            s3_client.download_file(s3_bucket_name, s3_key, f.name)
            total_topic_messages = 0
            bag = rosbag.Bag(f.name)
            for topic, msg, _ in bag.read_messages():
                if topic == topic_name:
                    total_topic_messages += 1
            self.assertEquals(total_topic_messages, total_test_messages)
    
    def publish_periodic_data_to_topic(self, topic, interval, total_messages):
        publisher = rospy.Publisher(topic, String, queue_size=total_messages)
        for _ in range(total_messages):
            msg = create_random_word(64) 
            publisher.publish(msg)
            time.sleep(interval)

def create_random_word(length):
    return ''.join([random.choice(string.ascii_letters + string.digits) for _ in range(length)])


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestDurationRecorderGeneral, sys.argv)
