# UploaderResult

# Error Codes
uint8 SUCCESS=0
# This includes service errors, or any error
# where the service was unreachable
uint8 UPLOAD_SERVICE_ERROR=1
# This includes invalid credentials, invalid destination
# or an invalid configuration
uint8 UPLOAD_FAILED_INVALID_INPUT=2

# an Error Code from the upload service
uint8 error_code

# Message describing the error
string message