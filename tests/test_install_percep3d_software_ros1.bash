#!/bin/bash
# =================================================================================================
# Test script for Percep3d software installer (ROS1 version)
#
# Usage:
#   $ bash test_install_percep3d_software_ros1.bash 
#
# Exit codes:
#   0 - Installation test completed successfully 
#   Non-zero - Installation test failed
# =================================================================================================

# Get repository root path and change to it
P3DS_PATH=$(git rev-parse --show-toplevel)
cd "${P3DS_PATH}" || exit 1

docker pull ubuntu:20.04

docker build \
        --build-arg BASE_IMAGE=ubuntu:20.04 \
        -f src/vm_software_install_ros1/Dockerfile.test \
        -t perce3d-software-ros1:noetic-full-ubuntu-20.04 \
        .
DOCKER_BUILD_EXIT_CODE=$?

if [[ ${DOCKER_BUILD_EXIT_CODE:?err} != 0 ]]; then
  echo -e "‼️  FAILED install_percep3d_software_ros1.bash testing"
  exit "${DOCKER_BUILD_EXIT_CODE}"
else
  echo -e "✅  Testing install_percep3d_software_ros1.bash completed with SUCCESS"
  exit 0
fi

