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

import random
import string
import tempfile

def create_temp_files(total_files):
    temp_file_names = []
    for _ in range(total_files):
        temp_file_names.append(create_temp_file())
    return temp_file_names

def create_temp_file():
    with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as temp_file:
        file_contents = ''.join(
            [random.choice(string.ascii_letters + string.digits + ' ') for _ in range(64)])
        temp_file.write(file_contents)
    return temp_file.name

def create_large_temp_file(file_size_in_mb):
    temp_file = tempfile.NamedTemporaryFile(suffix=".txt", delete=False)
    temp_file.seek(file_size_in_mb * 1024 * 1024 - 1)
    temp_file.write(b'0')
    temp_file.seek(0)
    temp_file.close()
    return temp_file.name