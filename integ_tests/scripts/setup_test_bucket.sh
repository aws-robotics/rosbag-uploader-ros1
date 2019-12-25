#!/bin/bash
BUCKET_NAME="awsrobomaker-integ-test-$(uuidgen)"
echo >&2
echo "Generated integration test bucket name: ${BUCKET_NAME}" >&2
echo >&2
printf ${BUCKET_NAME}
