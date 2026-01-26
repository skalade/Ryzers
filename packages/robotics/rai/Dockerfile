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

# Create virtual environment
ENV VIRTUAL_ENV=/opt/rai-venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install Poetry
RUN pip install --upgrade pip poetry

# Clone and install RAI framework
WORKDIR /opt
RUN git clone https://github.com/RobotecAI/rai.git

WORKDIR /opt/rai
RUN poetry config virtualenvs.create false \
 && poetry install --no-interaction --no-ansi

# Ensure venv is activated in shells
RUN echo "source $VIRTUAL_ENV/bin/activate" >> /root/.bashrc

# Copy test scripts
WORKDIR /ryzers
COPY test.sh /ryzers/test_rai.sh
RUN chmod +x /ryzers/test_rai.sh

# Expose common ports (Streamlit frontend, LangFuse)
EXPOSE 8501

# Default command runs test
CMD /ryzers/test_rai.sh
