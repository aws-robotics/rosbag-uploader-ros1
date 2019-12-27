# ROSBag Uploader Node - Integration Tests

End-to-End test of ROSBag uploader node functionality. 
This integration test package interacts with actual AWS services. 

## How to use? 
As they interact with actual AWS services, the ability to run these tests as a part of colcon tests have been disabled.
These tests are intended to be run with [rostest](http://wiki.ros.org/rostest). 

### Build 
`$ colcon build --packages-select rosbag_uploader_ros1_integration_tests`

### Run tests
* `$ source install/local_setup.bash`
* `$ rostest rosbag_uploader_ros1_integration_tests test_rosbag_uploader_ros1.test`
