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
 && rm -rf /var/lib/apt/lists/*

# Install Poetry for Python dependency management
RUN pip install --break-system-packages poetry

# Clone and install RAI framework
WORKDIR /opt
RUN git clone https://github.com/RobotecAI/rai.git

WORKDIR /opt/rai
RUN poetry config virtualenvs.create false \
 && poetry install --no-interaction --no-ansi

# Copy test scripts
WORKDIR /ryzers
COPY test.sh /ryzers/test_rai.sh
RUN chmod +x /ryzers/test_rai.sh

# Expose common ports (Streamlit frontend, LangFuse)
EXPOSE 8501

# Default command runs test
CMD /ryzers/test_rai.sh
