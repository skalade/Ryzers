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

import inspect
import robosuite
from robosuite.environments.base import MujocoEnv
print(f"robosuite version: {robosuite.__version__}")
# CaP-X requires the uynitsuj/robosuite fork that adds the
# `skip_render_images=` kwarg to MujocoEnv.step. Vanilla robosuite from PyPI
# does not have it, and trials crash at the first sim step.
assert "skip_render_images" in inspect.signature(MujocoEnv.step).parameters, (
    "robosuite is missing the CaP-X fork's `skip_render_images` kwarg on "
    "MujocoEnv.step -- did vanilla robosuite get pulled in?"
)
print("robosuite fork OK (skip_render_images kwarg present)")

import mujoco
print(f"mujoco version: {mujoco.__version__}")

import gymnasium
print(f"gymnasium version: {gymnasium.__version__}")

# A handful of the heavier transitive deps -- if these import without errors
# the env is healthy enough to run the upstream eval harness.
import transformers, ray, fastapi, viser, trimesh
print(f"transformers version: {transformers.__version__}")
print(f"ray version: {ray.__version__}")

# PyRoKi powers the privileged/oracle perception server profile.
import jax
import pyroki
from capx.serving import launch_pyroki_server
print(f"jax version: {jax.__version__} (devices={jax.devices()})")
print(f"pyroki loaded from: {pyroki.__file__}")
print(f"launch_pyroki_server loaded from: {launch_pyroki_server.__file__}")

# T1-B SAM3-on-ROCm shim: cap-x's launch_sam3_server.py is replaced at
# build time by a HuggingFace transformers-based implementation that runs
# on ROCm/CPU. Verify the replacement loaded (and not the upstream sam3
# CUDA fork) by checking the module docstring.
from capx.serving import launch_sam3_server
assert "ROCm-compatible" in (launch_sam3_server.__doc__ or ""), (
    "launch_sam3_server is not the ROCm replacement -- did the COPY in "
    "the Dockerfile not run?"
)
from transformers import Sam3Model, Sam2Model  # noqa: F401
print(f"launch_sam3_server (ROCm shim) loaded from: {launch_sam3_server.__file__}")
print("transformers Sam3Model / Sam2Model: OK (weights load on first /segment call)")

import numpy as np
assert np.__version__.startswith("1."), \
    f"numpy must stay on 1.x for robosuite/mink compatibility, got {np.__version__}"
print(f"numpy version: {np.__version__}")
PY

echo "Tests passed!"
