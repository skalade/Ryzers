#!/bin/bash

# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Smoke test for the SAM3-on-ROCm shim that replaces cap-x's CUDA-only
# launch_sam3_server. Starts the FastAPI server, runs a /segment_point
# round-trip on a synthetic image (which uses ungated SAM2 weights, no HF
# auth required), then shuts down.
#
# Run:
#   ryzers run /ryzers/demo_sam3_capx.sh
#
# Optional: to also exercise /segment (text-prompt SAM3 segmentation), set
# HF_TOKEN to a HuggingFace token that has accepted the
# https://huggingface.co/facebook/sam3 license.
#
# For low-contrast scenes (e.g. spill_wipe's 'brown spill' on a wood-grain
# table) export SAM3_THRESHOLD=0.1 to lower SAM3's instance-segmentation
# confidence floor. Default is 0.5, which suffices for high-contrast
# objects like 'red cube' / 'green cube' on the cube_stack scene.

set -euo pipefail

DEVICE=${SAM3_DEVICE:-cuda}  # ROCm builds of torch expose ROCm devices through the cuda namespace
PORT=${SAM3_PORT:-8114}

echo "Starting SAM3-on-ROCm shim on device=$DEVICE port=$PORT..."
python3 -m capx.serving.launch_sam3_server \
    --device "$DEVICE" --port "$PORT" \
    --no-eager-load-sam2 --no-eager-load-sam3 \
    > /tmp/sam3_demo_server.log 2>&1 &
SAM3_PID=$!
trap 'kill -TERM $SAM3_PID 2>/dev/null || true; wait $SAM3_PID 2>/dev/null || true' EXIT

# Wait for the server to bind
for i in $(seq 1 30); do
    if curl -sf "http://127.0.0.1:$PORT/health" > /dev/null 2>&1; then break; fi
    sleep 0.5
done

echo
echo "--- /health ---"
curl -s "http://127.0.0.1:$PORT/health" | python3 -m json.tool

echo
echo "--- /segment_point round-trip (ungated SAM2 weights) ---"
python3 - "$PORT" <<'PY'
import base64, io, sys, time
import numpy as np
import requests
from PIL import Image

port = sys.argv[1]

img = np.zeros((96, 96, 3), dtype=np.uint8)
img[28:68, 28:68, :] = 230
pil = Image.fromarray(img)
buf = io.BytesIO(); pil.save(buf, format="PNG")
b64 = base64.b64encode(buf.getvalue()).decode()

t0 = time.perf_counter()
resp = requests.post(
    f"http://127.0.0.1:{port}/segment_point",
    json={"image_base64": b64, "point_coords": [48.0, 48.0]},
    timeout=300,
)
elapsed = time.perf_counter() - t0
resp.raise_for_status()
data = resp.json()
print(f"  status={resp.status_code}, round-trip={elapsed:.2f}s")
print(f"  num_masks={len(data['scores'])}, top_score={max(data['scores']):.3f}")
print(f"  masks_shape={data['masks_shape']}, dtype={data['masks_dtype']}")
PY

echo
if [ -n "${HF_TOKEN:-}" ]; then
  echo "--- /segment text-prompt round-trip (gated SAM3 weights) ---"
  python3 - "$PORT" <<'PY'
import base64, io, sys, time
import numpy as np
import requests
from PIL import Image

port = sys.argv[1]
img = np.zeros((128, 128, 3), dtype=np.uint8)
img[40:88, 40:88, :] = 200
pil = Image.fromarray(img)
buf = io.BytesIO(); pil.save(buf, format="PNG")
b64 = base64.b64encode(buf.getvalue()).decode()

t0 = time.perf_counter()
resp = requests.post(
    f"http://127.0.0.1:{port}/segment",
    json={"image_base64": b64, "text_prompt": "bright square"},
    timeout=600,
)
elapsed = time.perf_counter() - t0
print(f"  status={resp.status_code}, round-trip={elapsed:.2f}s")
if resp.status_code == 200:
    data = resp.json()
    print(f"  num_results={len(data['results'])}")
    for r in data["results"][:3]:
        print(f"    label={r['label']!r}, score={r['score']:.3f}, box={r['box']}")
else:
    print(f"  failed: {resp.text}")
PY
else
  echo "HF_TOKEN not set; skipping /segment (text-prompt) test (facebook/sam3 weights are gated)."
  echo "To enable, run with: HF_TOKEN=hf_... ryzers run /ryzers/demo_sam3_capx.sh"
fi

echo
echo "SAM3-on-ROCm demo complete."
