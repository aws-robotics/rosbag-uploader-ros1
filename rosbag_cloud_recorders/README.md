## Rosbag Cloud Recorders

This package contains nodes that facilitate the recording of rosbag files.
The `rolling_recorder` node provides an action interface for uploading the past *x* minutes of rosbag files.
The `duration_recorder` node provides an action interface to record rosbags for a specified duration.
Once that duration is complete the rosbag files are uploaded to S3.
For more information on actions see the [`actionlib` documentation](http://wiki.ros.org/actionlib).
Examples for using the action servers can be found below.
The action servers can only take one request at a time.
If the node is currently working on a request it will reject any new requests.

### AWS Credentials

You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services.
You may find [AWS Configuration and Credential Files] helpful.

This node will require the following AWS account IAM role permissions:
- `s3:PutObject`
for the bucket specified in the config file.


## Usage

### Resource Setup

- [Create a bucket](https://docs.aws.amazon.com/AmazonS3/latest/gsg/CreatingABucket.html) in Amazon S3.

### Running the nodes

- Build the `rosbag_cloud_recorders` package as described in the top level README.
- Configure AWS credentials.
- Launch the `duration_recorder` and `s3_file_uploader` nodes with

        roslaunch rosbag_cloud_recorders duration_recorder_sample.launch s3_bucket:=<BUCKET_NAME>
- OR Launch the `rolling_recorder` and `s3_file_uploader` nodes with

        roslaunch rosbag_cloud_recorders rolling_recorder_sample.launch s3_bucket:=<BUCKET_NAME>

More details on launch parameters below.

### Example Action Client

A simple example of an action client to interact with this node is provided.
After sourcing the ROS workspace, the example client can be run with `python examples/recorder_client.py <node_type>`, where `<node_type>` can be `rolling_recorder` or `duration_recorder`.


## `duration_recorder` node

### Launch and Configuration File Parameters

| Name | Type | Description | Default Value |
| ---- | ---- | ----------- | ------------- |
| `min_free_disk_space` | int | The minimum amount of free disk space in MiB (the current action goal will be aborted when the free disk space falls below this amount) | 1024 |
| `write_directory` | string | The local directory where rosbags will be recorded (please make sure it is a writeable directory) | ~/.ros/dr_rosbag_uploader/ |
| `upload_timeout` | int | The time in seconds to wait for upload to complete | 3600 |
| `delete_bags_after_upload` | bool | Whether or not the bag files should be deleted after they have been successfully uploaded | false |

### Actions

**Action Name**: ~/DurationRecorder

**Goal**
| Key | Type | Description |
| --- | ---- | ----------- |
| `destination` | string | The S3 Key prefix of the rosbag files to be uploaded |
| `duration` | duration | The duration of time to record the rosbag for |
| `topics_to_record` | string[] | List of topics to record (If empty, all topics will be recorded) |

**Result**
| Key | Type | Description |
| --- | ---- | ----------- |
| `result` | uint8 | The return code associated with the goal |
| `message` | string | A message describing the reason for the result |

*Note* goals also have a message field that will contain more details on the result of the action

**Feedback**
| Key | Type | Description |
| --- | ---- | ----------- |
| `started` | time | The time at which this feedback was published; the time of entering the current stage |
| `stage` | uint8 | The stage of operation of the `rolling_recorder` action server |


## `rolling_recorder` node

### Launch and Configuration File Parameters

| Name | Type | Description | Default Value |
| ---- | ---- | ----------- | ------------- |
| `bag_rollover_time` | int | The length of time in seconds to be recorded per bag file | 30 |
| `max_record_time` | int | The length of time recordings should be kept in rosbags, which will be uploaded when requested (older rosbags will have been deleted and not be uploaded) | 300 |
| `min_free_disk_space` | int | The minimum amount of free disk space in MiB (the node will error out when the free disk space falls below this amount) | 1024 |
| `topics_to_record` (configuration file parameter only) | string[] | List of topics that should be recorded to rosbags | *empty* (all active topics will be recorded) |
| `write_directory` | string | The local directory where rosbags will be recorded (please make sure it is a writeable directory) | ~/.ros/rr_rosbag_uploader/ |
| `upload_timeout` | int | The time in seconds to wait for upload to complete | 3600 |

### Actions

**Action Name**: ~/RollingRecorder

**Goal**
| Key | Type | Description |
| --- | ---- | ----------- |
| `destination` | string | The S3 Key prefix of the rosbag files to be uploaded |

**Result**
| Key | Type | Description |
| --- | ---- | ----------- |
| `result` | uint8 | The return code associated with the goal |
| `message` | string | A message describing the reason for the result |

*Note* goals also have a message field that will contain more details on the result of the action

**Feedback**
| Key | Type | Description |
| --- | ---- | ----------- |
| `started` | time | The time at which this feedback was published; the time of entering the current stage |
| `stage` | uint8 | The stage of operation of the `rolling_recorder` action server |


[AWS Configuration and Credential Files]: https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html
