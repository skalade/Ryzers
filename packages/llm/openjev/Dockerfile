# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# OpenJev decision server. The base image is vllm/vllm-openai-rocm:v0.29.0
# (set in config.yaml). That image's PyTorch is built for gfx1151, so it
# covers Ryzen AI MAX / AI 300. The published image ENTRYPOINT is
# `vllm serve`; clear it so `ryzers run` can execute the test or serve script.

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

ENTRYPOINT []

# Pins from openjev/openjev serve/SERVE.md. A constraint keeps the image's
# ROCm torch in place: an unconstrained transformers install can replace it
# with a CUDA wheel from PyPI.
RUN torch_spec="$(python3 -c 'import torch; print(torch.__version__)')" && \
    printf 'torch==%s\n' "${torch_spec}" > /tmp/torch-constraint.txt && \
    python3 -m pip install --no-cache-dir \
        --index-url https://pypi.org/simple \
        -c /tmp/torch-constraint.txt \
        "openai==3.16.2" \
        "httpx==0.28.1" \
        "transformers==5.17.0" && \
    python3 -c "import torch; assert torch.__version__ == '${torch_spec}', torch.__version__; assert torch.version.hip, torch.__version__; print('torch', torch.__version__, 'hip', torch.version.hip)"

# Helper only. The bfloat16 weights are about 54 GB and are downloaded
# at serve time into the mounted model directory.
ENV OPENJEV_ROOT=/opt/openjev
RUN mkdir -p "${OPENJEV_ROOT}/model" && \
    python3 - <<'PY'
from huggingface_hub import hf_hub_download
hf_hub_download(
    repo_id="openjev/openjev",
    filename="helper/shim.py",
    local_dir="/opt/openjev",
)
PY
RUN echo "81a22f1b1b8912a465059207ef9f60b7c6c16b4de6372305d867efbe38a1987a  ${OPENJEV_ROOT}/helper/shim.py" | sha256sum -c -

WORKDIR /ryzers
COPY test.sh /ryzers/test_openjev.sh
COPY serve.sh /ryzers/serve_openjev.sh
RUN chmod +x /ryzers/test_openjev.sh /ryzers/serve_openjev.sh

EXPOSE 3000

CMD ["/ryzers/test_openjev.sh"]
