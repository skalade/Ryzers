# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

WORKDIR /ryzers

# Install deps
RUN apt-get update && apt-get install -y git vim wget

# Install mujoco separately since no cpython wheel for python3.12
RUN apt-get update && apt-get install -y \
    cmake \
    ca-certificates \
    wget \
    patchelf \
    libgl1-mesa-dev \
    libosmesa6-dev \
    libglfw3 \
    libglew-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
 && rm -rf /var/lib/apt/lists/*

ARG MUJOCO_VERSION=2.3.7
ENV MUJOCO_PATH=/root/.mujoco/mujoco-${MUJOCO_VERSION}
ENV MUJOCO_PLUGIN_PATH=/root/.mujoco/plugins-empty
ENV LD_LIBRARY_PATH=${MUJOCO_PATH}/bin:${LD_LIBRARY_PATH}

# Download and install mujoco
RUN mkdir -p /root/.mujoco \
 && wget -q https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz -O /root/.mujoco/mujoco.tar.gz \
 && tar -xzf /root/.mujoco/mujoco.tar.gz -C /root/.mujoco \
 && rm /root/.mujoco/mujoco.tar.gz \
 && mkdir -p "${MUJOCO_PLUGIN_PATH}"

# Install mujoco
RUN pip install --no-cache-dir mujoco==${MUJOCO_VERSION}

# Install openpi-specific lerobot
RUN pip install git+https://github.com/huggingface/lerobot.git@0cf864870cf29f4738d3ade893e6fd13fbd7cdb5#0cf864870cf29f4738d3ade893e6fd13fbd7cdb5

# Install ROCm jax
RUN pip install jax[rocm]

# Setup openpi
RUN git clone https://github.com/Physical-Intelligence/openpi && \
    cd openpi && \
    git submodule update --init --recursive

RUN cd openpi && \
    pip install -e packages/openpi-client/ && \
    pip install -e .

COPY test.py /ryzers/test.py
COPY test.sh /ryzers/test.sh
RUN chmod +x /ryzers/test.sh

CMD ["/bin/bash", "-c", "/ryzers/test.sh"]
