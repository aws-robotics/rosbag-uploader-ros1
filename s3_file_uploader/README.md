## Amazon S3 Uploader

The S3FileUploader node provides an action interface for uploading a set of files to Amazon S3. For more information on actions see the (actionlib documentation)[http://wiki.ros.org/actionlib]. Examples for using the UploadFiles action server can be found below. The action server can take one upload request at a time. If the uploader is currently working on an upload request it will reject any new requests.

### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files] helpful.

This node will require the following AWS account IAM role permissions:
- `s3:PutObject`
for the bucket specified in the config file.


## Usage
### Resource Setup
- (Create a bucket)[https://docs.aws.amazon.com/AmazonS3/latest/gsg/CreatingABucket.html] in Amazon S3.

### Running the node

- Build the s3_file_uploader package following the instructions in the main README.
- Configure AWS credentials.
- Change the node configuration with the S3 Bucket that files should be uploaded to. Note that this bucket must be in the same
region as in the node configuration.
- Launch the node with

        roslaunch s3_file_uploader s3_file_uploader.launch


### Simple Upload File Test
A simple example of a client to interact with this node:
```
import actionlib
import rospy

from file_uploader_msgs.msg import UploadFilesAction, UploadFilesGoal

ACTION = "/s3_file_uploader/UploadFiles"
FILE_NAME = "/tmp/hello_world.txt"
NODE_NAME = "s3_file_uploader_client"
S3_KEY_PREFIX = "rosbags/test"

rospy.init_node(NODE_NAME)

goal = UploadFilesGoal(
            upload_location=S3_KEY_PREFIX,
            files=[FILE_NAME]
        )
client = actionlib.SimpleActionClient(ACTION, UploadFilesAction)
client.wait_for_server()
client.send_goal(goal)
client.wait_for_result(rospy.Duration.from_sec(15.0))
print(client.get_result())

# If the configured bucket is called my-bucket then after this action is completed the contents of /tmp/hello_world.txt
# wil be available at s3://my-bucket/rosbags/test/hello_world.txt
```

The action server can also be invoked via the command line
```
 rostopic pub  /s3_file_uploader/UploadFiles/goal   file_uploader_msgs/UploadFilesActionGoal "{goal: { files:['/tmp/hello_world.txt'], upload_location: 'rosbags/test' } }"
```

## s3_file_uploader node
### Actions
**Action Name**: ~/UploadFiles

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


[AWS Configuration and Credential Files]: https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html
