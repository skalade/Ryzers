#!/usr/bin/env bash
# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Test for the CaP-X / ROCm image. Three stages:
#   1. Sign-of-life: ROCm torch sees the GPU; the stack + simulator import;
#      MuJoCo renders headless via EGL.
#   2. Oracle eval (ALWAYS runs, no LLM/keys): runs a full CaP-Gym episode with
#      the environment's ground-truth `oracle_code` through the real sim +
#      PyRoKi IK pipeline and asserts task success (reward 1.0). This is the
#      deterministic correctness check for the eval pipeline on ROCm.
#   3. LLM eval (OPTIONAL): if CAPX_LLM_SERVER_URL is set, runs a short real
#      agentic eval (LLM writes code -> sim executes -> reward) against any
#      OpenAI-compatible endpoint. See the package README for how to point this
#      at OpenRouter or a local/on-prem server (vLLM-ROCm, llama.cpp).
set -euo pipefail

CAPX_SIM="${CAPX_SIM:-robosuite}"
if [ "$CAPX_SIM" = "libero" ]; then
    ORACLE_ENV="franka_libero_pick_place_code_env_privileged"
    LLM_CONFIG="env_configs/libero/franka_libero_spatial_0.yaml"
else
    ORACLE_ENV="franka_robosuite_pick_place_code_env"
    LLM_CONFIG="env_configs/cube_stack/franka_robosuite_cube_stack.yaml"
fi

cd /ryzers/cap-x

echo "================ [1/3] CaP-X / ROCm sign-of-life ================"
lsb_release -a 2>/dev/null || true
python3 -c "
import torch
print(f'PyTorch : {torch.__version__}')
print(f'ROCm/HIP: {torch.version.hip}')
print(f'GPU ok  : {torch.cuda.is_available()}')
if torch.version.hip is None:
    raise SystemExit('This is not a ROCm/HIP PyTorch build')
if torch.cuda.is_available():
    for i in range(torch.cuda.device_count()):
        print(f'  device {i}: {torch.cuda.get_device_name(i)}')
else:
    raise SystemExit('No ROCm GPU visible - check /dev/kfd, /dev/dri and HSA_OVERRIDE_GFX_VERSION')
"
# jax is CPU-only by design (matches CaP-X's lock: jax 0.4.29, no GPU plugin).
python3 -c "
import capx, sam3, contact_graspnet_pytorch
import jax; print('jax', jax.__version__, 'devices (CPU expected):', jax.devices())
import pyroki, jaxls
print('capx + pyroki import OK')
"
if [ "$CAPX_SIM" = "libero" ]; then
    python3 -c "import libero, robosuite; print('LIBERO + robosuite import OK')"
else
    python3 -c "import robosuite; print('robosuite', robosuite.__version__, 'import OK')"
fi
MUJOCO_GL=egl python3 -c "
import os; os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type=\"box\" size=\".1 .1 .1\"/></worldbody></mujoco>')
d = mujoco.MjData(m)
r = mujoco.Renderer(m, 64, 64)
try:
    mujoco.mj_forward(m, d)
    r.update_scene(d)
    print('EGL render OK, frame shape', r.render().shape)
finally:
    r.close()
"

echo "================ [2/3] Oracle eval (no LLM): ${ORACLE_ENV} ================"
# The oracle path runs the full sim + control pipeline using ground-truth code.
# It only needs the PyRoKi IK server (no LLM, no SAM3/GraspNet).
PYROKI_PID=""
cleanup_pyroki() {
    if [ -n "${PYROKI_PID:-}" ] && kill -0 "$PYROKI_PID" 2>/dev/null; then
        kill "$PYROKI_PID" 2>/dev/null || true
        wait "$PYROKI_PID" 2>/dev/null || true
    fi
}
trap cleanup_pyroki EXIT

python3 capx/serving/launch_pyroki_server.py \
    --port 8116 --robot panda_description --target-link panda_hand \
    > /tmp/pyroki.log 2>&1 &
PYROKI_PID=$!
echo "waiting for PyRoKi IK server..."
PYROKI_READY=0
for i in $(seq 1 60); do
    if curl -sf http://127.0.0.1:8116/ik -X POST -H "Content-Type: application/json" \
        -d '{"target_pose_wxyz_xyz":[1,0,0,0,0.4,0,0.3]}' >/dev/null 2>&1; then
        echo "PyRoKi ready (warmed JAX JIT) after ${i}s"
        PYROKI_READY=1
        break
    fi
    sleep 1
done
if [ "$PYROKI_READY" -ne 1 ]; then
    echo "PyRoKi failed to become ready"
    cat /tmp/pyroki.log
    exit 1
fi

if ! ORACLE_OUT=$(timeout 400 python3 tests/test_environments.py --env-name "${ORACLE_ENV}" 2>&1); then
    echo "ORACLE EVAL FAILED"
    tail -20 <<<"$ORACLE_OUT"
    exit 1
fi
grep -aE "Time taken:|Reward:|Success" <<<"$ORACLE_OUT" | tail -4 || true
cleanup_pyroki
trap - EXIT
if grep -aq '^Success$' <<<"$ORACLE_OUT"; then
    echo "ORACLE EVAL PASSED (reward 1.0)"
else
    echo "ORACLE EVAL FAILED"
    tail -20 <<<"$ORACLE_OUT"
    exit 1
fi

echo "================ [3/3] LLM eval (optional) ================"
if [ -z "${CAPX_LLM_SERVER_URL:-}" ]; then
    echo "SKIPPED — set CAPX_LLM_SERVER_URL (and CAPX_LLM_MODEL) to run a real"
    echo "agentic eval against an OpenAI-compatible endpoint. See README."
else
    MODEL="${CAPX_LLM_MODEL:-gemma4}"
    TRIALS="${CAPX_LLM_TRIALS:-2}"
    echo "Running ${TRIALS}-trial eval: model=${MODEL} server=${CAPX_LLM_SERVER_URL}"
    echo "config=${LLM_CONFIG}"
    LLM_LOG=$(mktemp)
    if ! timeout 1800 python3 capx/envs/launch.py \
            --config-path "${LLM_CONFIG}" \
            --model "${MODEL}" \
            --server-url "${CAPX_LLM_SERVER_URL}" \
            --total-trials "${TRIALS}" \
            --num-workers 1 \
            --max-tokens 4096 >"$LLM_LOG" 2>&1; then
        echo "LLM EVAL FAILED"
        tail -40 "$LLM_LOG"
        rm -f "$LLM_LOG"
        exit 1
    fi
    grep -aE "Trial [0-9]+ took|reward_|task_completed|Time taken to query" "$LLM_LOG" | tail -12 || true
    rm -f "$LLM_LOG"
    echo "LLM EVAL COMPLETE (see outputs/<model>/ for per-trial rewards + videos)"
fi

echo "================ CaP-X tests PASSED ================"
