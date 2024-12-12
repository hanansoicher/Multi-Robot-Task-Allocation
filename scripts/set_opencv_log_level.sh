#!/bin/bash
# This script sets the OPENCV_LOG_LEVEL environment variable to suppress warnings

export OPENCV_LOG_LEVEL="ERROR"
echo "OPENCV_LOG_LEVEL set to $OPENCV_LOG_LEVEL"