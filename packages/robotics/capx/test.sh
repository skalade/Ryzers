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

# PyRoKi powers the privileged/oracle perception server profile -- the only
# api_servers profile that doesn't require CUDA-only SAM3 / Contact-GraspNet.
import jax
import pyroki
from capx.serving import launch_pyroki_server
print(f"jax version: {jax.__version__} (devices={jax.devices()})")
print(f"pyroki loaded from: {pyroki.__file__}")
print(f"launch_pyroki_server loaded from: {launch_pyroki_server.__file__}")

import numpy as np
assert np.__version__.startswith("1."), \
    f"numpy must stay on 1.x for robosuite/mink compatibility, got {np.__version__}"
print(f"numpy version: {np.__version__}")
PY

echo "Tests passed!"
