#!/bin/bash

# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Download openjev/openjev if needed, serve it with the measured vLLM flags,
# and run the upstream decision helper on port 3000.
# vLLM binds 127.0.0.1. The helper binds 127.0.0.1 unless OPENJEV_SHIM_HOST
# is set. A non-loopback helper host requires SHIM_TOKEN. See the README.

set -euo pipefail
set -m

MODEL_DIR="${OPENJEV_MODEL_DIR:-/opt/openjev/model}"
VLLM_PORT="${OPENJEV_VLLM_PORT:-8000}"
SHIM_PORT="${OPENJEV_SHIM_PORT:-3000}"
SHIM_HOST="${OPENJEV_SHIM_HOST:-127.0.0.1}"
GPU_MEM="${OPENJEV_GPU_MEMORY_UTILIZATION:-0.90}"
SHIM=/opt/openjev/helper/shim.py

mkdir -p "$MODEL_DIR"

if [[ ! -f "${MODEL_DIR}/config.json" ]]; then
  echo "Downloading openjev/openjev into ${MODEL_DIR} (about 54 GB)..."
  OPENJEV_MODEL_DIR="$MODEL_DIR" python3 - <<'PY'
import os
from huggingface_hub import snapshot_download
snapshot_download(repo_id="openjev/openjev", local_dir=os.environ["OPENJEV_MODEL_DIR"])
PY
fi

export VLLM="${VLLM:-http://127.0.0.1:${VLLM_PORT}/v1}"
export TOKENIZER="${TOKENIZER:-$MODEL_DIR}"
export READOUT_T="${READOUT_T:-0.85}"
export READOUT_NOUL_T="${READOUT_NOUL_T:-1.829074}"
export READOUT_NOUL_BIAS="${READOUT_NOUL_BIAS:-0}"
export READOUT_TARGETED="${READOUT_TARGETED:-1}"
export READOUT_INSTR_STYLE="${READOUT_INSTR_STYLE:-pyrepr}"
export SHIM_STAGGER="${SHIM_STAGGER:-1}"

if [[ "$SHIM_HOST" != "127.0.0.1" && -z "${SHIM_TOKEN:-}" ]]; then
  echo "Refusing to bind the helper to ${SHIM_HOST} without SHIM_TOKEN."
  exit 1
fi

VLLM_PID=""
cleanup() {
  if [[ -n "$VLLM_PID" ]]; then
    kill -- -"$VLLM_PID" 2>/dev/null || kill "$VLLM_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "Starting vLLM on 127.0.0.1:${VLLM_PORT}..."
vllm serve "$MODEL_DIR" \
  --host 127.0.0.1 \
  --served-model-name qwen \
  --port "$VLLM_PORT" \
  --enable-prefix-caching \
  --max-model-len 16384 \
  --gpu-memory-utilization "$GPU_MEM" \
  --limit-mm-per-prompt '{"image":1}' \
  --trust-remote-code \
  --max-num-seqs 256 \
  --max-logprobs 64 \
  --gdn-prefill-backend triton \
  --quantization fp8 \
  > /tmp/openjev-vllm.log 2>&1 &
VLLM_PID=$!

echo "Waiting for vLLM (log: /tmp/openjev-vllm.log)..."
ready=0
for _ in $(seq 1 600); do
  if curl -sf "http://127.0.0.1:${VLLM_PORT}/v1/models" >/dev/null; then
    ready=1
    break
  fi
  if ! kill -0 "$VLLM_PID" 2>/dev/null; then
    echo "vLLM exited before it was ready."
    cat /tmp/openjev-vllm.log
    exit 1
  fi
  sleep 2
done

if [[ "$ready" -ne 1 ]]; then
  echo "vLLM did not become ready in time."
  tail -n 80 /tmp/openjev-vllm.log || true
  exit 1
fi

echo "Decision API at http://${SHIM_HOST}:${SHIM_PORT}/v1/systemone"
python3 "$SHIM" --host "$SHIM_HOST" --port "$SHIM_PORT"
