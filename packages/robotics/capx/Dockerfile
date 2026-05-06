# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    HSA_OVERRIDE_GFX_VERSION=11.0.0 \
    MUJOCO_GL=egl \
    PYOPENGL_PLATFORM=egl

# System deps for MuJoCo / Robosuite / EGL rendering, FFmpeg, Vulkan, and
# building any small C extensions our pip install pulls in.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl ca-certificates locales \
    build-essential cmake patchelf \
    libgl1 libglib2.0-0 libglfw3 \
    libglew-dev libosmesa6-dev \
    libegl1 libgles2 \
    libvulkan1 mesa-vulkan-drivers \
    ffmpeg \
 && rm -rf /var/lib/apt/lists/*

RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && locale-gen

# CaP-X targets CUDA upstream. On ROCm we keep the base image's ROCm-built
# PyTorch and install only the CPU/ROCm-safe subset of CaP-X's deps via pip
# (no uv, no editable submodules). The CUDA-only optional extras --
# sam3, nvidia-curobo, contact_graspnet_pytorch, flash-attn, vllm, verl --
# are intentionally skipped.
RUN pip3 install --no-cache-dir --break-system-packages \
    "numpy<2" \
    "gymnasium>=0.29" \
    "mujoco>=3.2.0" \
    "robosuite>=1.5.0" \
    "tyro>=0.8.5" \
    "pydantic>=2.7" \
    "rich>=13.7" \
    "scipy>=1.12" \
    "matplotlib>=3.10" \
    "imageio[ffmpeg]>=2.34" \
    "requests" \
    "tqdm" \
    "h5py" \
    "einops" \
    "pycocotools" \
    "pyyaml" \
    "omegaconf" \
    "trimesh>=4.4.0" \
    "robot_descriptions>=1.16.0" \
    "yourdfpy>=0.0.56" \
    "viser>=0.2.0" \
    "msgpack_numpy" \
    "ray==2.48.0" \
    "mediapy>=1.2.5" \
    "uvicorn" \
    "fastapi" \
    "transformers<5.0" \
    "openai" \
    "pyrender" \
    "opencv-python-headless<4.13" \
    "pillow>=10" \
    "cloudpickle" \
    "open3d" \
    "PyOpenGL>=3.1.7"

WORKDIR /ryzers

# Clone CaP-X without --recurse-submodules. The submodules (sam3, curobo,
# contact_graspnet_pytorch, b1k/Isaac-Sim, verl, LIBERO-PRO and the cap-x
# robosuite forks) are CUDA-bound, NVIDIA-only, or supplant the wheel deps
# above; we don't need any of them for ROCm Robosuite evaluation.
ARG CAPX_REF=v0.0.1
RUN git clone https://github.com/capgym/cap-x.git /ryzers/cap-x \
 && cd /ryzers/cap-x \
 && git checkout ${CAPX_REF}

# Editable install with --no-deps so pip doesn't try to resolve the
# pyproject.toml entries (which include sam3, nvidia-curobo, etc.). All
# runtime deps were installed explicitly above.
RUN pip3 install --no-cache-dir --break-system-packages --no-deps -e /ryzers/cap-x

COPY test.sh /ryzers/test_capx.sh
RUN chmod +x /ryzers/test_capx.sh

# LLM-proxy port (8110) and Web UI port (8200) from the upstream Quick Start.
EXPOSE 8110
EXPOSE 8200

CMD /ryzers/test_capx.sh
