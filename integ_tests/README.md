## ROSBag Uploader Node - Integration Tests

End-to-End test of ROSBag uploader node functionality. 
This integration test package interacts with actual AWS services. 


## How to use? 

As these tests interact with actual AWS services, the ability to run these tests as a part of `colcon test` has been disabled.
These tests are intended to be run with [rostest](http://wiki.ros.org/rostest).
It also follows that valid AWS credentials are required to run these tests successfully.
The credentials will need to have the permissions to create/delete/read/write S3 buckets and objects.

### Build 

`$ colcon build --packages-select rosbag_uploader_ros1_integration_tests`

### Run tests

* Setup the necessary AWS credentials
* `$ source install/local_setup.bash`
* `$ rostest rosbag_uploader_ros1_integration_tests <rostest file>`
