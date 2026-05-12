#!/bin/bash

# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# CaP-X "no LLM" smoke demo. Runs a single oracle-code trial of the
# Robosuite Franka cube-stack task using the privileged perception
# server profile (PyRoKi only). No OpenRouter / vLLM / API key needed.
#
# Run:
#   ryzers run /ryzers/demo_capx.sh
#
# Expected: a single trial completes with reward 1.0 / task_completed: True
# in roughly 35-50 seconds. The first run downloads ~25s of robot URDFs
# into /root; subsequent runs are warmer.

set -e

cd /ryzers/cap-x

OUTPUT_DIR=${OUTPUT_DIR:-/tmp/capx_oracle_demo}

python3 capx/envs/launch.py \
    --config-path env_configs/cube_stack/franka_robosuite_cube_stack_privileged.yaml \
    --use-oracle-code True \
    --total-trials 1 \
    --num-workers 1 \
    --record-video False \
    --output-dir "$OUTPUT_DIR"

echo
echo "Demo complete. Trial artifacts in $OUTPUT_DIR/oracle/"
