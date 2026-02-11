# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

WORKDIR /ryzers

# Install system deps
RUN apt-get update && apt-get install -y git vim wget build-essential

# Clone repo with submodules (calvin_env, LIBERO, pyhash, etc.)
RUN git clone --recurse-submodules https://github.com/intuitive-robots/flower_vla_calvin.git

WORKDIR /ryzers/flower_vla_calvin

# Install calvin_env submodule deps (tacto first, then calvin_env)
RUN cd calvin_env/tacto && pip install -e .
RUN cd calvin_env && pip install -e .

# Skip pyhash-0.9.3: incompatible with Python 3.12 (use_2to3 removed from setuptools,
# distutils removed from stdlib). Only needed for dataset hashing, not model inference.

# Install main requirements (remove torch pin to keep base image's torch,
# remove hydra pin so we can install a Python 3.12 compatible version)
RUN sed -i '/^torch==/d' requirements.txt && \
	sed -i '/^hydra-core/d' requirements.txt && \
	pip install -r requirements.txt

# Upgrade hydra to a version compatible with Python 3.12
RUN pip install 'hydra-core>=1.3'

# Install flower package
RUN pip install -e .

# Pin transformers<4.50 — Florence-2 remote code (trust_remote_code=True) breaks on >=4.50
# with 'forced_bos_token_id' AttributeError. Also install extras for model download.
RUN pip install 'transformers>=4.46,<4.50' huggingface_hub safetensors

WORKDIR /ryzers

COPY test.py /ryzers/test.py
COPY test.sh /ryzers/test.sh
RUN chmod +x /ryzers/test.sh

CMD ["/bin/bash", "-c", "/ryzers/test.sh"]
