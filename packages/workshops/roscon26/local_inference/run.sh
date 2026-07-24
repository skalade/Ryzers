#!/bin/bash
# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Runs the local inference image. With --network=host, JupyterLab (8888) and the
# Lemonade API (13305) are reachable directly on the host. Pass a command
# (e.g. /ryzers/test_rai.sh) to override the default JupyterLab launch.

docker run -it --rm \
  --shm-size 16G \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  --network=host \
  --ipc=host \
  -e HSA_OVERRIDE_GFX_VERSION=11.0.0 \
  -v $PWD/notebooks:/ryzers/notebooks \
  -v $PWD/workspace/.cache/huggingface:/root/.cache/huggingface \
  -v $PWD/workspace/.cache/lemonade:/root/.cache/lemonade \
  --device=/dev/kfd \
  --device=/dev/dri \
  --group-add video \
  --group-add render \
  roscon26_local_inference "$@"
