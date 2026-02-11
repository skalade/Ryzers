# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

WORKDIR /ryzers

# Install system deps
RUN apt-get update && apt-get install -y git vim wget build-essential

# Clone repo with submodules
RUN git clone --recurse-submodules https://github.com/intuitive-robots/flower_vla_calvin.git

WORKDIR /ryzers/flower_vla_calvin

# Install calvin_env submodule deps (tacto first, then calvin_env)
RUN cd calvin_env/tacto && pip install -e .
RUN cd calvin_env && pip install -e .

# Skip pyhash-0.9.3: incompatible with Python 3.12 (use_2to3 removed from setuptools,
# distutils removed from stdlib). Only needed for dataset hashing, not model inference.

# Install main requirements (keep base image torch,
# remove hydra pin so we can install a Python 3.12 compatible version)
RUN sed -i '/^torch==/d' requirements.txt && \
	sed -i '/^hydra-core/d' requirements.txt && \
	pip install -r requirements.txt

# Upgrade hydra to a version compatible with Python 3.12
RUN pip install 'hydra-core>=1.3'

# Newer PyTorch forbids passing both attn_mask and is_causal=True to SDPA.
# Fix by checking `mask is None` (the computed mask) instead of `custom_attn_mask is None`.
RUN sed -i 's/is_causal=is_causal if custom_attn_mask is None else False/is_causal=is_causal if mask is None else False/' \
	flower/models/networks/transformers.py

# Install flower package
RUN pip install -e .

# Pin transformers<4.50 — Florence-2 remote code (trust_remote_code=True) breaks on >=4.50
# with 'forced_bos_token_id' AttributeError. Also install extras for model download.
RUN pip install 'transformers>=4.46,<4.50' huggingface_hub safetensors

WORKDIR /ryzers

COPY test.py /ryzers/flower_vla_calvin/test.py
COPY test.sh /ryzers/test_flower_vla.sh
RUN chmod +x /ryzers/test_flower_vla.sh

CMD ["/bin/bash", "-c", "/ryzers/test_flower_vla.sh"]
