# UploaderResult

# Result Codes
uint8 SUCCESS=0
# This includes uploader service errors, invalid credentials,
# or any error where the service was unreachable
uint8 UPLOAD_FAILED_SERVICE_ERROR=1
# This includes invalid destination, invalid files
uint8 UPLOAD_FAILED_INVALID_INPUT=2


# A status code from the uploader service
uint8 result

# Message describing the status
string message

# The list of files that were uploaded
string[] files_uploaded