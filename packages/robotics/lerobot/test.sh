#!/usr/bin/env bash
# Copyright (C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

set -e

# Patch test training script to run quickly
sed -i 's/training_steps = 5000/training_steps = 2/' /ryzers/lerobot/examples/training/train_policy.py
sed -i 's/batch_size=64/batch_size=4/' /ryzers/lerobot/examples/training/train_policy.py

# Run Test
python /ryzers/lerobot/examples/training/train_policy.py