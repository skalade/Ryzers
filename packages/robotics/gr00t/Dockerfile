# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

WORKDIR /ryzers

# Install deps
RUN apt-get update && apt-get install -y git wget rocm-dev hipcc

RUN pip install zmq diffusers decord

# Build pytorch3d since there's no wheel for python3.12
ENV FORCE_CUDA=1
RUN pip install --no-build-isolation "git+https://github.com/facebookresearch/pytorch3d.git"

# Install flash attention
ENV FLASH_ATTENTION_TRITON_AMD_ENABLE="TRUE" 
ENV BUILD_TYPE="rocm"
RUN git clone --recursive https://github.com/ROCm/flash-attention && \
    cd flash-attention && \
    git checkout main_perf && \
    python setup.py install

RUN git clone https://github.com/NVIDIA/Isaac-GR00T && \
    cd Isaac-GR00T && \
    pip install -e .

COPY test.py /ryzers/test.py
COPY test.sh /ryzers/test.sh
RUN chmod +x /ryzers/test.sh

CMD ["/bin/bash", "-c", "/ryzers/test.sh"]
