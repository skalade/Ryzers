#!/bin/bash
# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Runs the MolmoAct2 finetuning image. With --network=host, JupyterLab (8888) is
# reachable directly on the host. Pass a command (e.g. python /ryzers/test.py) to
# override the default JupyterLab launch. Model weights download once into the
# mounted Hugging Face cache.

docker run -it --rm \
  --shm-size 16G \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  --network=host \
  --ipc=host \
  -e HF_HOME=/root/.cache/huggingface \
  -e HF_TOKEN=${HF_TOKEN:-} \
  -v $PWD/notebooks:/ryzers/notebooks \
  -v $PWD/workspace/.cache/huggingface:/root/.cache/huggingface \
  --device=/dev/kfd \
  --device=/dev/dri \
  --group-add video \
  --group-add render \
  roscon26_finetuning "$@"
