#!/bin/bash

# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Multi-task CaP-X benchmark on ROCm, no API key required.
#
# Runs N trials of oracle-code on the seven Robosuite privileged-config tasks
# we know to work on this image. Reports per-trial rewards and per-task wall
# time. Useful as both a smoke test for the eval harness and a real (if small)
# benchmark of the oracle-code success rate.
#
# Run:
#   ryzers run /ryzers/bench_capx.sh                 # default: 3 trials/task
#   TRIALS=10 ryzers run /ryzers/bench_capx.sh       # override
#   TASKS=cube_stack,nut_assembly ryzers run /ryzers/bench_capx.sh   # subset

set -euo pipefail

cd /ryzers/cap-x

TRIALS=${TRIALS:-3}
DEFAULT_TASKS="cube_stack:env_configs/cube_stack/franka_robosuite_cube_stack_privileged.yaml \
cube_lifting:env_configs/cube_lifting/franka_robosuite_cube_lifting_privileged.yaml \
cube_restack:env_configs/cube_restack/franka_robosuite_cube_restack_privileged.yaml \
nut_assembly:env_configs/nut_assembly/franka_robosuite_nut_assembly_privileged.yaml \
spill_wipe:env_configs/spill_wipe/franka_robosuite_spill_wipe_privileged.yaml \
two_arm_lift:env_configs/two_arm_lift/franka_robosuite_two_arm_lift_privileged.yaml \
two_arm_handover:env_configs/two_arm_handover/two_arm_handover_privileged.yaml"

# If TASKS is set (comma-separated), filter to those names.
if [ -n "${TASKS:-}" ]; then
    SELECTED=""
    for entry in $DEFAULT_TASKS; do
        name="${entry%%:*}"
        for want in $(echo "$TASKS" | tr ',' ' '); do
            if [ "$name" = "$want" ]; then SELECTED="$SELECTED $entry"; fi
        done
    done
    TASKS_RUN="$SELECTED"
else
    TASKS_RUN="$DEFAULT_TASKS"
fi

OUTPUT_ROOT=${OUTPUT_DIR:-/tmp/capx_bench}
rm -rf "$OUTPUT_ROOT"
mkdir -p "$OUTPUT_ROOT"

echo "=========================================="
echo "CaP-X oracle-code benchmark on ROCm"
echo "  trials/task: $TRIALS"
echo "  output:      $OUTPUT_ROOT"
echo "=========================================="

declare -A SUCCESS
declare -A TOTAL
declare -A WALL

for entry in $TASKS_RUN; do
    name="${entry%%:*}"
    cfg="${entry##*:}"
    echo
    echo "--- $name ($cfg, $TRIALS trials) ---"

    t0=$(date +%s.%N)
    python3 capx/envs/launch.py \
        --config-path "$cfg" \
        --use-oracle-code True \
        --total-trials "$TRIALS" \
        --num-workers 1 \
        --record-video False \
        --output-dir "$OUTPUT_ROOT/$name" 2>&1 \
      | grep -E '^  (Reward|Task Completed)' \
      | tee "$OUTPUT_ROOT/$name.log"
    t1=$(date +%s.%N)

    s=$(grep -c "Task Completed: True" "$OUTPUT_ROOT/$name.log" || true)
    SUCCESS[$name]=$s
    TOTAL[$name]=$TRIALS
    WALL[$name]=$(awk -v t0="$t0" -v t1="$t1" 'BEGIN{printf "%.1f", t1-t0}')
done

echo
echo "=========================================="
echo "Summary (oracle-code, no LLM, on ROCm)"
echo "=========================================="
printf "%-22s %-12s %-12s\n" task success wall_s
total_succ=0; total_runs=0
for entry in $TASKS_RUN; do
    name="${entry%%:*}"
    s=${SUCCESS[$name]:-0}
    n=${TOTAL[$name]:-0}
    w=${WALL[$name]:-0}
    printf "%-22s %-12s %-12s\n" "$name" "$s/$n" "$w"
    total_succ=$((total_succ + s))
    total_runs=$((total_runs + n))
done
printf "%-22s %-12s\n" "TOTAL" "$total_succ/$total_runs"
