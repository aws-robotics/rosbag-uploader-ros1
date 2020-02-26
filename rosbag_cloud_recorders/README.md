## Rosbag Cloud Recorders

This package contains two nodes. The RollingRecorder node provides an action interface for uploading the past x minutes of rosbag files. The DurationRecorder provides an action interface to record rosbags for a specified duration. Once that duration is complete the rosbag files are uploaded to S3. For more information on actions see the (actionlib documentation)[http://wiki.ros.org/actionlib]. Examples for using the action servers can be found below. The action servers can only take one request at a time. If the node is currently working on a request it will reject any new requests.

### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files] helpful.

This node will require the following AWS account IAM role permissions:
- `s3:PutObject`
for the bucket specified in the config file.


## Usage
### Resource Setup
- (Create a bucket)[https://docs.aws.amazon.com/AmazonS3/latest/gsg/CreatingABucket.html] in Amazon S3.

### Running the nodes
- Build the rosbag_cloud_recorders package as described in the top level README.
- Configure AWS credentials.
- Launch the DurationRecorder with

        roslaunch rosbag_cloud_recorders duration_recorder_sample.launch s3_bucket:=<BUCKET_NAME>
- OR Launch the RollingRecorder with

        roslaunch rosbag_cloud_recorders rolling_recorder_sample.launch s3_bucket:=<BUCKET_NAME>

### Example Action Client
A simple example of a client to interact with this node. This can be run with `python examples/recorder_client.py <node_type>` after sourcing the ros workspace where `node_type` can be `rolling_recorder` or `duration_recorder`

## rolling_recorder node
### Actions
**Action Name**: ~/RosbagRollingRecord

**Goal**

| Key | Type | Description |
| --- | ---- | ----------- |
| files | string[] | A list of absolute paths to files to be uploaded. Note that these paths must be accessible from the s3_file_uploader node |
| upload_location | string | The S3 Key prefix |

**Result**

| Key | Type | Description |
| --- | ---- | ----------- |
| result_code | uint16 | The error code returned from S3. The enum list can be found (here)[https://sdk.amazonaws.com/cpp/api/LATEST/_s3_errors_8h_source.html] |
| files_uploaded | string[] | The list of files that were successfully uploaded. |

*Note* goals also have a message field that will contain more details on the result of the action


**Feedback**
| Key | Type | Description |
| --- | ---- | ----------- |
| num_uploaded | uint16 | The number of files that have been uploaded from this request so far. |
| num_remaining | uint16 | The number of files that are remaining in this upload request. |

## duration_recorder node
### Actions
**Action Name**: ~/RosbagDurationRecord

**Goal**

| Key | Type | Description |
| --- | ---- | ----------- |
| destination | string | The S3 Key prefix for the uploaded rosbags |
| duration | duration |  How long to record the rosbag for. Must be positive|
| topics_to_record | string[] | List of topics to record. If empty will record all topics |

**Result**

| Key | Type | Description |
| --- | ---- | ----------- |
| result_code | uint16 | The error code returned from S3. The enum list can be found (here)[https://sdk.amazonaws.com/cpp/api/LATEST/_s3_errors_8h_source.html] |
| files_uploaded | string[] | The list of files that were successfully uploaded. |

*Note* goals also have a message field that will contain more details on the result of the action


**Feedback**
| Key | Type | Description |
| --- | ---- | ----------- |
| num_uploaded | uint16 | The number of files that have been uploaded from this request so far. |
| num_remaining | uint16 | The number of files that are remaining in this upload request. |

[AWS Configuration and Credential Files]: https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html