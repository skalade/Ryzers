#!/bin/bash

docker build -t roscon_npu --build-arg ROS_DISTRO=kilted -f Dockerfile.npu .
