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

import rostest

from file_helpers import create_temp_file
from s3_file_uploader_test_base import S3FileUploaderTestBase

PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'test_s3_file_uploader_wrong_region'
AWS_REGION = 'eu-west-1'
INCORRECT_REGION_RESULT_CODE = 100 # Currently incorrect region returns UNKNOWN(100)

class TestS3FileUploaderWrongRegion(S3FileUploaderTestBase):
    @classmethod
    def extract_s3_region(cls):
        return AWS_REGION

    def test_bucket_in_wrong_region(self):
        self._create_upload_files_action_client()
        temp_file_name = create_temp_file()
        self.files_to_delete.append(temp_file_name)
        result = self._upload_temp_files([temp_file_name])
        self.assertFalse(result.result_code.success)
        self.assertEqual(result.result_code.error_code, INCORRECT_REGION_RESULT_CODE,
            "Result code was %d" % result.result_code.error_code)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestS3FileUploaderWrongRegion, sys.argv)
