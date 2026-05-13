# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

"""ROCm-compatible drop-in replacement for capx/serving/launch_sam3_server.py.

Upstream cap-x ships a SAM3 server that imports a CUDA-only fork
(Max-Fu/sam3) which builds C++ extensions against CUDA_HOME at install
time and fails on ROCm. This file delegates the same /segment and
/segment_point endpoints to HuggingFace transformers' pure-PyTorch
implementations:

    /segment        -> transformers.Sam3Model        (text-prompt segmentation)
    /segment_point  -> transformers.Sam2Model        (point-prompt segmentation)

Sam3 (transformers >=5.0) does not expose a point-prompt API the way
Max-Fu's Sam3 fork does, so we route point prompts through Sam2 instead.
Both are pure PyTorch and run on ROCm using the base image's torch build
with HSA_OVERRIDE_GFX_VERSION=11.0.0 (Ryzen AI iGPU shim).

Notes:
    - facebook/sam3 weights are gated; users must pre-authenticate with
      `huggingface-cli login` (or set HF_TOKEN) before /segment will load.
    - facebook/sam2.1-hiera-large is not gated; /segment_point works
      out of the box.
    - Models are loaded lazily on first request so the server can start
      and serve /segment_point even if the gated SAM3 weights are not
      available on the host yet.
    - JSON I/O contract is identical to the upstream Max-Fu/sam3 server,
      so capx/integrations/vision/sam3.py needs no client-side changes.
"""

from __future__ import annotations

import asyncio
import base64
import functools
import io
import logging
import os
import threading
from typing import Any, List

import numpy as np
import torch
import tyro
import uvicorn
from fastapi import FastAPI, HTTPException
from PIL import Image
from pydantic import BaseModel


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI()

_DEVICE: str = "cuda"  # PyTorch ROCm builds expose ROCm devices via the cuda namespace
_GPU_SEMAPHORE: asyncio.Semaphore | None = None  # serialize GPU access; created at startup
_LOAD_LOCK = threading.Lock()

_SAM3_MODEL_NAME = os.environ.get("SAM3_MODEL", "facebook/sam3")
_SAM2_MODEL_NAME = os.environ.get("SAM2_MODEL", "facebook/sam2.1-hiera-large")

# Lazy holders -- loaded on first call, then cached for the process lifetime.
_SAM3: dict[str, Any] | None = None
_SAM2: dict[str, Any] | None = None


def _ensure_sam3() -> dict[str, Any]:
    global _SAM3
    if _SAM3 is not None:
        return _SAM3
    with _LOAD_LOCK:
        if _SAM3 is not None:
            return _SAM3
        try:
            from transformers import Sam3Model, Sam3Processor  # type: ignore[attr-defined]
        except ImportError as e:
            raise HTTPException(
                status_code=503,
                detail=(
                    "transformers is too old to expose Sam3Model "
                    "(need transformers>=5.0). Got: " + str(e)
                ),
            )
        logger.info("Lazy-loading Sam3 model %s on %s", _SAM3_MODEL_NAME, _DEVICE)
        try:
            model = Sam3Model.from_pretrained(_SAM3_MODEL_NAME).to(_DEVICE).eval()
            processor = Sam3Processor.from_pretrained(_SAM3_MODEL_NAME)
        except Exception as e:
            raise HTTPException(
                status_code=503,
                detail=(
                    f"Failed to load Sam3 weights from {_SAM3_MODEL_NAME}. "
                    f"facebook/sam3 is gated -- accept the license at "
                    f"https://huggingface.co/facebook/sam3 and run "
                    f"`huggingface-cli login` before starting the server. "
                    f"Underlying error: {e}"
                ),
            )
        _SAM3 = {"model": model, "processor": processor}
        logger.info("Sam3 ready")
        return _SAM3


def _ensure_sam2() -> dict[str, Any]:
    global _SAM2
    if _SAM2 is not None:
        return _SAM2
    with _LOAD_LOCK:
        if _SAM2 is not None:
            return _SAM2
        try:
            from transformers import Sam2Model, Sam2Processor  # type: ignore[attr-defined]
        except ImportError as e:
            raise HTTPException(
                status_code=503,
                detail=(
                    "transformers is too old to expose Sam2Model "
                    "(need transformers>=4.50). Got: " + str(e)
                ),
            )
        logger.info("Lazy-loading Sam2 model %s on %s", _SAM2_MODEL_NAME, _DEVICE)
        model = Sam2Model.from_pretrained(_SAM2_MODEL_NAME).to(_DEVICE).eval()
        processor = Sam2Processor.from_pretrained(_SAM2_MODEL_NAME)
        _SAM2 = {"model": model, "processor": processor}
        logger.info("Sam2 ready")
        return _SAM2


async def _run_blocking(fn, *args, **kwargs):
    assert _GPU_SEMAPHORE is not None
    async with _GPU_SEMAPHORE:
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, functools.partial(fn, *args, **kwargs))


def decode_image(base64_str: str) -> Image.Image:
    try:
        image_data = base64.b64decode(base64_str)
        return Image.open(io.BytesIO(image_data)).convert("RGB")
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid image data: {e}")


def encode_mask(mask: np.ndarray) -> str:
    return base64.b64encode(mask.astype(np.uint8).tobytes()).decode("utf-8")


def encode_array(arr: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(arr).tobytes()).decode("utf-8")


# --- Request/Response Models (must match upstream JSON contract) ---


class SegmentRequest(BaseModel):
    image_base64: str
    text_prompt: str


class PointPromptRequest(BaseModel):
    image_base64: str
    point_coords: list[float]  # [x, y]


class PointPromptResponse(BaseModel):
    scores: list[float]
    masks_base64: str
    masks_shape: list[int]
    masks_dtype: str


class MaskData(BaseModel):
    mask_base64: str
    shape: list[int]
    box: list[float]
    score: float
    label: str


class SegmentResponse(BaseModel):
    results: list[MaskData]


# --- Core inference paths ---


def _do_segment(pil_image: Image.Image, text_prompt: str) -> SegmentResponse:
    sam3 = _ensure_sam3()
    model = sam3["model"]
    processor = sam3["processor"]

    inputs = processor(images=pil_image, text=text_prompt, return_tensors="pt").to(_DEVICE)
    autocast_dtype = torch.bfloat16 if "cuda" in _DEVICE else torch.float32
    autocast_device = "cuda" if "cuda" in _DEVICE else "cpu"
    with torch.no_grad(), torch.autocast(autocast_device, dtype=autocast_dtype):
        outputs = model(**inputs)

    target_sizes = inputs.get("original_sizes")
    if target_sizes is not None:
        target_sizes = target_sizes.tolist()
    results_list = processor.post_process_instance_segmentation(
        outputs,
        threshold=0.5,
        mask_threshold=0.5,
        target_sizes=target_sizes,
    )
    if not results_list:
        return SegmentResponse(results=[])
    out = results_list[0]

    masks = out.get("masks")
    boxes = out.get("boxes")
    scores = out.get("scores")
    if masks is None or boxes is None or scores is None:
        return SegmentResponse(results=[])

    masks_np = masks.detach().cpu().numpy()
    boxes_np = boxes.detach().cpu().numpy()
    scores_np = scores.detach().cpu().float().numpy()
    if masks_np.ndim == 4 and masks_np.shape[1] == 1:
        masks_np = masks_np.squeeze(1)

    results: List[MaskData] = []
    for i in range(len(scores_np)):
        mask = (masks_np[i] > 0).astype(bool)
        results.append(
            MaskData(
                mask_base64=encode_mask(mask),
                shape=list(mask.shape),
                box=[float(v) for v in boxes_np[i].tolist()],
                score=float(scores_np[i]),
                label=text_prompt,
            )
        )
    results.sort(key=lambda r: r.score, reverse=True)
    return SegmentResponse(results=results)


def _do_segment_point(
    pil_image: Image.Image, point_coords: tuple[float, float]
) -> PointPromptResponse:
    sam2 = _ensure_sam2()
    model = sam2["model"]
    processor = sam2["processor"]

    x, y = float(point_coords[0]), float(point_coords[1])
    inputs = processor(
        images=pil_image,
        input_points=[[[[x, y]]]],  # batch=1, num_objs=1, num_points=1, xy
        input_labels=[[[1]]],  # foreground point
        return_tensors="pt",
    ).to(_DEVICE)

    autocast_dtype = torch.bfloat16 if "cuda" in _DEVICE else torch.float32
    autocast_device = "cuda" if "cuda" in _DEVICE else "cpu"
    with torch.no_grad(), torch.autocast(autocast_device, dtype=autocast_dtype):
        outputs = model(**inputs, multimask_output=True)

    target_sizes = inputs.get("original_sizes")
    if target_sizes is not None:
        target_sizes = target_sizes.tolist()
    masks = processor.post_process_masks(
        outputs.pred_masks.cpu(),
        original_sizes=target_sizes,
        binarize=False,
    )[0]
    scores = outputs.iou_scores.cpu().float().numpy().reshape(-1)
    masks_np = masks.detach().cpu().float().numpy()
    if masks_np.ndim == 4 and masks_np.shape[0] == 1:
        masks_np = masks_np[0]

    if masks_np.size == 0 or scores.size == 0:
        return PointPromptResponse(
            scores=[],
            masks_base64="",
            masks_shape=[0, 0, 0],
            masks_dtype="float32",
        )

    sort_idx = np.argsort(scores)[::-1]
    masks_np = masks_np[sort_idx]
    scores = scores[sort_idx]
    masks_np = masks_np.astype(np.float32)

    return PointPromptResponse(
        scores=[float(s) for s in scores],
        masks_base64=encode_array(masks_np),
        masks_shape=list(masks_np.shape),
        masks_dtype=str(masks_np.dtype),
    )


# --- Endpoints ---


@app.get("/health")
def health() -> dict[str, Any]:
    return {
        "ok": True,
        "device": _DEVICE,
        "sam3_loaded": _SAM3 is not None,
        "sam2_loaded": _SAM2 is not None,
        "sam3_model": _SAM3_MODEL_NAME,
        "sam2_model": _SAM2_MODEL_NAME,
        "backend": "transformers (ROCm/CPU)",
    }


@app.post("/segment", response_model=SegmentResponse)
async def segment(req: SegmentRequest) -> SegmentResponse:
    pil_image = decode_image(req.image_base64)
    try:
        return await _run_blocking(_do_segment, pil_image, req.text_prompt)
    except HTTPException:
        raise
    except Exception as e:
        logger.exception("/segment failed")
        raise HTTPException(status_code=500, detail=f"Inference failed: {e}")


@app.post("/segment_point", response_model=PointPromptResponse)
async def segment_point(req: PointPromptRequest) -> PointPromptResponse:
    if len(req.point_coords) != 2:
        raise HTTPException(
            status_code=400,
            detail=f"point_coords must be [x, y], got {req.point_coords!r}",
        )
    pil_image = decode_image(req.image_base64)
    try:
        return await _run_blocking(
            _do_segment_point, pil_image, (req.point_coords[0], req.point_coords[1])
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.exception("/segment_point failed")
        raise HTTPException(status_code=500, detail=f"Inference failed: {e}")


def main(
    device: str = "cuda",
    port: int = 8114,
    host: str = "127.0.0.1",
    eager_load_sam2: bool = True,
    eager_load_sam3: bool = False,
):
    global _DEVICE, _GPU_SEMAPHORE
    _DEVICE = device

    # Ampere/Hopper-style TF32 toggles are no-ops on ROCm but harmless.
    if torch.cuda.is_available():
        torch.backends.cuda.matmul.allow_tf32 = True
        try:
            torch.backends.cudnn.allow_tf32 = True
        except Exception:
            pass

    _GPU_SEMAPHORE = asyncio.Semaphore(1)

    if eager_load_sam2:
        try:
            _ensure_sam2()
        except HTTPException as e:
            logger.warning("Sam2 eager-load failed; will retry on first request: %s", e.detail)
    if eager_load_sam3:
        try:
            _ensure_sam3()
        except HTTPException as e:
            logger.warning("Sam3 eager-load failed; will retry on first request: %s", e.detail)

    logger.info("SAM3-on-ROCm shim listening on %s:%s (device=%s)", host, port, device)
    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    tyro.cli(main)
