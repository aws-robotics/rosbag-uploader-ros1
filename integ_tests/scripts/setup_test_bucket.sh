#!/bin/bash
UUID=$(python -c 'import sys,uuid; sys.stdout.write(uuid.uuid4().hex[:12])')
BUCKET_NAME="awsrobomaker-integ-test-$UUID"
echo >&2
echo "Generated integration test bucket name: ${BUCKET_NAME}" >&2
echo >&2
printf ${BUCKET_NAME}
