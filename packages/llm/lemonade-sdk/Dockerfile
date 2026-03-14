# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    wget \
    git \
 && rm -rf /var/lib/apt/lists/*

# Install lemonade-sdk and dependencies
RUN pip3 install --no-cache-dir --break-system-packages \
    "lemonade-sdk" \
    "matplotlib"

# Install llamacpp backend (Vulkan)
RUN lemonade-install --llamacpp vulkan

# Optionally install NPU support (XRT/XDNA + FastFlowLM)
ARG ENABLE_NPU="false"
ARG FLM_VERSION="0.9.35"
RUN if [ "$ENABLE_NPU" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            software-properties-common \
     && add-apt-repository -y ppa:amd-team/xrt \
     && apt-get update \
     && apt-get install -y --no-install-recommends \
            libxrt-npu2 \
            libxrt-utils-npu \
     && wget -q -O /tmp/fastflowlm.deb \
            https://github.com/FastFlowLM/FastFlowLM/releases/download/v${FLM_VERSION}/fastflowlm_${FLM_VERSION}_ubuntu24.04_amd64.deb \
     && apt-get install -y /tmp/fastflowlm.deb \
     && rm -f /tmp/fastflowlm.deb \
     && rm -rf /var/lib/apt/lists/*; \
    fi

# Copy test script
WORKDIR /ryzers
COPY test.sh /ryzers/test_lemonade-sdk.sh
RUN chmod +x /ryzers/test_lemonade-sdk.sh

EXPOSE 8000

CMD /ryzers/test_lemonade-sdk.sh
