#!/bin/bash

# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

set -euo pipefail

echo "Running tests for openjev..."

SHIM=/opt/openjev/helper/shim.py
EXPECTED_SHA=81a22f1b1b8912a465059207ef9f60b7c6c16b4de6372305d867efbe38a1987a

if [[ ! -f "$SHIM" ]]; then
  echo "FAIL: helper not found at $SHIM"
  exit 1
fi

echo "${EXPECTED_SHA}  ${SHIM}" | sha256sum -c -
python3 -m py_compile "$SHIM"

python3 - <<'PY'
import httpx
import openai
import torch
import transformers
import vllm

assert vllm.__version__.startswith("0.29.0"), vllm.__version__
assert openai.__version__ == "3.16.2", openai.__version__
assert httpx.__version__ == "0.28.1", httpx.__version__
assert transformers.__version__ == "5.17.0", transformers.__version__
assert torch.version.hip, torch.__version__

print(f"vllm {vllm.__version__}")
print(f"torch {torch.__version__} hip {torch.version.hip}")
print(f"transformers {transformers.__version__}")
print(f"openai {openai.__version__}")
print(f"httpx {httpx.__version__}")
PY

if ! command -v vllm >/dev/null; then
  echo "FAIL: vllm is not on PATH"
  exit 1
fi

echo "Tests passed!"
echo "Weights are not loaded by this test."
echo "Start the decision server with: ryzers run /ryzers/serve_openjev.sh"
