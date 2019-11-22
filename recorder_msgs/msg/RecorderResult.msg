# Recorder Result

# Result Codes
uint8 SUCCESS=0
uint8 ROSBAG_RECORDING_FAILED=1
# This includes service errors, or any error
# where the service was unreachable
uint8 UPLOAD_SERVICE_ERROR=2
# This includes invalid credentials, invalid destination
# or an invalid configuration
uint8 UPLOAD_FAILED_BAD_REQUEST=3

# One of the result codes defined above
uint8 result

# Message describing the error
string message