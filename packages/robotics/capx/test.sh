#!/bin/bash

# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

set -e

echo "Running tests for capx..."

python3 - <<'PY'
import importlib.metadata

import capx
print(f"capx version: {capx.__version__}")
print(f"capx loaded from: {capx.__file__}")

import torch
print(f"torch version: {torch.__version__}")
# Note: ROCm builds of PyTorch report ROCm devices through the cuda namespace.
print(f"torch GPU available: {torch.cuda.is_available()}")
if hasattr(torch.version, "hip") and torch.version.hip:
    print(f"torch ROCm/HIP version: {torch.version.hip}")

import robosuite
print(f"robosuite version: {robosuite.__version__}")

import mujoco
print(f"mujoco version: {mujoco.__version__}")

import gymnasium
print(f"gymnasium version: {gymnasium.__version__}")

# A handful of the heavier transitive deps -- if these import without errors
# the env is healthy enough to run the upstream eval harness.
import transformers, ray, fastapi, viser, trimesh
print(f"transformers version: {transformers.__version__}")
print(f"ray version: {ray.__version__}")
PY

echo "Tests passed!"
