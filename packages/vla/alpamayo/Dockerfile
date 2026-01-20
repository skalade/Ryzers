# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

WORKDIR /ryzers

# Install deps
RUN apt-get update && apt-get install -y git vim wget

# Install python deps
RUN pip install mediapy

# Clone and patch torch deps so rocm ones don't get overriden
RUN git clone https://github.com/NVlabs/alpamayo
RUN sed -i -E '/^\s*dependencies\s*=\s*\[/,/^\s*\]/{ /^\s*"(torch|torchvision|torchaudio)[^"]*",?\s*(#.*)?$/d }' alpamayo/pyproject.toml
RUN sed -i -E '/^\s*dependencies\s*=\s*\[/,/^\s*\]/{ /^\s*"(flash-attn)[^"]*",?\s*(#.*)?$/d }' alpamayo/pyproject.toml

RUN cd alpamayo && \
    pip install -e .

COPY test.py /ryzers/alpamayo/notebooks/test.py
COPY test.sh /ryzers/test.sh
RUN chmod +x /ryzers/test.sh

CMD ["/bin/bash", "-c", "/ryzers/test.sh"]
