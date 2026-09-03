#!/bin/bash
# Stop on errors
set -e

# Build the docker image
docker build -t openvins_realsense_humble -f docker/Dockerfile .
