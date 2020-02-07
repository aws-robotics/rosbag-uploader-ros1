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
import random
import string
import sys
import time

import rostest

from recorder_msgs.msg import DurationRecorderResult
from duration_recorder_test_base import DurationRecorderTestBase

PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'test_duration_record_general'
RESULT_CODE_SUCCESS = 0

class TestDurationRecorderGeneral(DurationRecorderTestBase):
    def test_record_duration(self):
        self._create_duration_recorder_action_client()
        start_time = time.time()
        action_result = self.record_for_duration(1)
        self.assertEquals(action_result.result.result, RESULT_CODE_SUCCESS)
        self.check_rosbags_were_recorded(start_time, 1)

    def test_record_multiple_times(self):
        self._create_duration_recorder_action_client()
        start_time = time.time()
        total_recordings = 5
        for _ in range(total_recordings):
            action_result = self.record_for_duration(1)
            self.assertEquals(action_result.result.result, RESULT_CODE_SUCCESS)
        self.check_rosbags_were_recorded(start_time, total_recordings)

    def test_record_specific_topic(self):
        self._create_duration_recorder_action_client()
        start_time = time.time()
        topic_name = '/my_random_topic_' + self.create_random_word(8) 

        # Start publishing messages on another thread
        initial_delay = 0.5
        interval = 0.1
        total_test_messages = 10
        pub_func = partial(self.publish_periodic_data_to_topic, topic_name, initial_delay, interval, total_test_messages)
        pub_thread = threading.Thread(name='pub_to_topic', target=pub_func)
        pub_thread.daemon = True
        pub_thread.start()

        action_result = self.record_for_duration(2, [topic_name])
        self.assertEquals(action_result.result.result, RESULT_CODE_SUCCESS)
        self.check_rosbags_were_recorded(start_time, 1)

        # Ensure that the rosbag contains all the test messages
        rosbag = self.get_latest_bag_by_regex("*.bag") 
        total_topic_messages = 0
        bag = rosbag.Bag(latest_bag)
        for topic, msg, _ in bag.read_messages():
            if topic == topic_name:
                total_topic_messages += 1
        self.assertEquals(total_topic_messages, total_test_messages)
    
    def publish_periodic_data_to_topic(self, topic, initial_delay, interval, total_messages):
        publisher = rospy.Publisher(topic, String, queue_size=total_messages)
        time.sleep(initial_delay)
        for _ in range(total_messages):
            msg = self.create_random_word() 
            publisher.publish(msg)
            time.sleep(interval)

    def create_random_word(self, length):
        ''.join([random.choice(string.ascii_letters + string.digits) for _ in range(length)])


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestDurationRecorderGeneral, sys.argv)
