# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

FROM rocm/pytorch:rocm7.2.2_ubuntu24.04_py3.12_pytorch_release_2.10.0

# iGPU groups (from ryzer_env)
RUN getent group render || groupadd render
RUN getent group video  || groupadd video
RUN getent group kvm    || groupadd kvm
RUN usermod -aG render root && usermod -aG video root && usermod -aG kvm root

ARG MOLMOACT2_COMMIT=cdf4b7728e0545366bf3366639fd86477ef59d04

WORKDIR /ryzers

ENV HF_HUB_ENABLE_HF_TRANSFER=0 \
    HF_HUB_DISABLE_XET=1 \
    HF_HUB_DISABLE_TELEMETRY=1 \
    TOKENIZERS_PARALLELISM=false \
    HF_HOME=/root/.cache/huggingface \
    TORCH_ROCM_AOTRITON_ENABLE_EXPERIMENTAL=1 \
    TORCH_BLAS_PREFER_HIPBLASLT=0

# MolmoAct2 server/inference deps EXCEPT torch/torchvision (cu121 wheels would
# clobber the ROCm torch already in the base image).
RUN pip install --no-cache-dir \
      "transformers>=4.57,<4.58" \
      "accelerate>=1.0" \
      "safetensors>=0.4" \
      "huggingface-hub[cli]>=0.36,<1.0" \
      "hf-transfer>=0.1.8" \
      "pillow>=10" \
      "numpy>=1.26,<3" \
      "einops>=0.7" \
      "sentencepiece>=0.2" \
      "protobuf>=4.25" \
      "fastapi>=0.116" \
      "uvicorn[standard]>=0.35" \
      "json-numpy>=2.1.0"

# Bake the upstream MolmoAct2 source at the pinned commit for use from notebooks.
RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && \
    apt-get install -y --no-install-recommends git && rm -rf /var/lib/apt/lists/* && \
    rm -rf /repos/molmoact2 && \
    git clone https://github.com/allenai/molmoact2 /repos/molmoact2 && \
    git -C /repos/molmoact2 checkout ${MOLMOACT2_COMMIT}

# Build-time guard: fail the build if anything swapped torch off ROCm.
RUN python -c "import torch; assert torch.version.hip, 'torch is not a ROCm build: '+torch.__version__; print('OK torch', torch.__version__, 'hip', torch.version.hip)"

# Individual ryzer tests (env sign-of-life, no weights required).
COPY test_torch.sh test_molmoact2.sh /ryzers/
RUN chmod +x /ryzers/test_*.sh

# ============================================================================
# Workshop: JupyterLab + notebooks
# ============================================================================
RUN pip install --no-cache-dir jupyterlab ipywidgets matplotlib

COPY notebooks /ryzers/notebooks/
RUN chmod -R 777 /ryzers/notebooks

RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'export SHELL=/bin/bash' >> /entrypoint.sh && \
    echo 'exec python3 -m jupyterlab --ip=0.0.0.0 --port=8888 --allow-root --no-browser --notebook-dir=/ryzers "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

EXPOSE 8888

CMD ["/bin/bash", "/entrypoint.sh"]
