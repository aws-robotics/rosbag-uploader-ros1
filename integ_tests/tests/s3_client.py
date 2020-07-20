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
import boto3

from botocore.client import Config
from botocore.exceptions import ClientError

_SIGV4_NAME = 's3v4'

class S3Client(object):
    def __init__(self, region_name):
        self.s3_client = boto3.client(
            's3', region_name=region_name, config=Config(signature_version=_SIGV4_NAME))
        self.s3_resource = boto3.resource('s3', region_name=region_name)
        self.s3_region = region_name

    def create_bucket(self, bucket_name):
        response = self.s3_client.create_bucket(
            Bucket=bucket_name,
            CreateBucketConfiguration={
                'LocationConstraint': self.s3_region
            }
        )
        return response['Location']

    def delete_bucket(self, bucket_name):
        self.s3_client.delete_bucket(
          Bucket=bucket_name
        )

    def delete_all_objects(self, bucket_name):
        bucket = self.s3_resource.Bucket(bucket_name)
        bucket.objects.all().delete()

    def delete_objects(self, bucket_name, keys):
        objects_to_delete = []
        for key in keys:
            objects_to_delete.append({
                'Key': key.strip()
            })
        self.s3_client.delete_objects(
            Bucket=bucket_name,
            Delete={
                'Objects': objects_to_delete
            }
        )

    def list_objects(self, bucket_name):
        response = self.s3_client.list_objects(Bucket=bucket_name)
        return response.get('Contents', [])

    def wait_for_bucket_create(self, bucket_name):
        waiter = self.s3_client.get_waiter('bucket_exists')
        waiter.wait(Bucket=bucket_name)

    def download_file(self, bucket_name, key, file_name):
        self.s3_client.download_file(
            Bucket=bucket_name,
            Key=key,
            Filename=file_name)
