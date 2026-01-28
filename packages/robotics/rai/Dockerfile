# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install additional dependencies for RAI
RUN apt-get update && apt-get install -y \
    git \
    ffmpeg \
    libportaudio2 \
    python3-venv \
 && rm -rf /var/lib/apt/lists/*

# Use existing virtual env from base image
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN echo "source $VIRTUAL_ENV/bin/activate" >> /root/.bashrc

# Install Poetry into the venv
RUN pip install --upgrade pip poetry empy lark

# Clone and install RAI framework
WORKDIR /ryzers
RUN git clone https://github.com/RobotecAI/rai.git

WORKDIR /ryzers/rai
RUN vcs import < ros_deps.repos
RUN poetry config virtualenvs.create false && \
    poetry install --all-groups

RUN colcon build --symlink-install

# Copy test scripts
WORKDIR /ryzers
COPY test.sh /ryzers/test_rai.sh
RUN chmod +x /ryzers/test_rai.sh

# Expose common ports (Streamlit frontend, LangFuse)
EXPOSE 8501

# Default command runs test
CMD /ryzers/test_rai.sh
