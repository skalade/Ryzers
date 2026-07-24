#!/bin/bash
# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

set -e

echo "Testing MolmoAct2 dependencies..."

python3 - <<'PY'
import sys
import transformers
import accelerate
import huggingface_hub
import einops          # noqa: F401
import fastapi         # noqa: F401
import json_numpy      # noqa: F401
import safetensors     # noqa: F401
import sentencepiece   # noqa: F401

print(f"transformers     : {transformers.__version__}")
if not transformers.__version__.startswith("4.57"):
    print(f"FAIL: want transformers 4.57.x, got {transformers.__version__}", file=sys.stderr)
    sys.exit(1)
print(f"accelerate       : {accelerate.__version__}")
print(f"huggingface_hub  : {huggingface_hub.__version__}")
print("deps import ok   : einops, fastapi, json_numpy, safetensors, sentencepiece")
print("PASS: MolmoAct2 deps OK")
PY
