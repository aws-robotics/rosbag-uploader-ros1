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

from file_helpers import create_temp_file, create_temp_files, create_large_temp_file
from s3_file_uploader_test_base import S3FileUploaderTestBase

PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'test_s3_file_uploader_general'
OVERSIZE_FILE_RESULT_CODE = 100 # Currently oversize file returns UNKNOWN(100)

class TestS3FileUploaderGeneral(S3FileUploaderTestBase):
    def test_upload_file(self):
        self._create_upload_files_action_client()
        temp_file_name = create_temp_file()
        self.files_to_delete.append(temp_file_name)
        result = self._upload_temp_files([temp_file_name])
        self._assert_successful_upload(result, [temp_file_name])

    def test_upload_multiple_files(self):
        self._create_upload_files_action_client()
        temp_file_names = create_temp_files(10)
        self.files_to_delete += temp_file_names
        result = self._upload_temp_files(temp_file_names)
        self._assert_successful_upload(result, temp_file_names)

    def test_upload_oversize_file(self):
        self._create_upload_files_action_client()
        # S3 Limit is 5GB, add a little extra to be sure
        file_size_in_mb = 5500
        temp_file_name = create_large_temp_file(file_size_in_mb)
        self.files_to_delete.append(temp_file_name)
        result = self._upload_temp_files([temp_file_name])
        self.assertFalse(result.result_code.success)
        self.assertEqual(result.result_code.error_code, OVERSIZE_FILE_RESULT_CODE,
            "Result code was %d" % result.result_code.error_code)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestS3FileUploaderGeneral, sys.argv)
