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

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestDurationRecorderGeneral, sys.argv)
