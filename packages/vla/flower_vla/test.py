#!/usr/bin/env python3
# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

import torch
import time

# Disable optimizations, currently trigger HIP errors
torch.backends.cuda.enable_flash_sdp(False)
torch.backends.cuda.enable_mem_efficient_sdp(False)

from omegaconf import OmegaConf
from huggingface_hub import hf_hub_download
from flower.models.flower import FLOWERVLA

MODEL_REPO = "mbreuss/flower_calvin_abc"

# Download model files from HuggingFace
print("\nDownloading FlowerVLA model files...")
config_path = hf_hub_download(MODEL_REPO, "config.yaml")
weights_path = hf_hub_download(MODEL_REPO, "model.safetensors")

# Load config and extract model section
full_cfg = OmegaConf.load(config_path)
kwargs = OmegaConf.to_container(full_cfg.model, resolve=True)

# Remove hydra args so FLOWERVLA constructor doesn't complain
kwargs.pop("_target_", None)
kwargs.pop("_recursive_", None)

# Override for inference
kwargs.update(
    load_pretrained=True,
    pretrained_model_path=weights_path,
)

# Instantiate model (~1B params, Florence-2-large backbone + DiT)
print("Instantiating FlowerVLA model...")
model = FLOWERVLA(**kwargs)
model = model.to("cuda").eval()

# Create dummy inputs: [batch, time_steps, channels, height, width]
dummy_img = torch.rand(1, 1, 3, 224, 224, device="cuda")
obs = {"rgb_obs": {"rgb_static": dummy_img, "rgb_gripper": dummy_img}}
goal = {"lang_text": "pick up the red block"}

# Warmup
print("Warming up...")
for _ in range(3):
    with torch.no_grad():
        actions = model(obs, goal)
torch.cuda.synchronize()

# Benchmark
print("Running benchmark...")
torch.cuda.reset_peak_memory_stats()
num_iterations = 100
latencies = []

for _ in range(num_iterations):
    torch.cuda.synchronize()
    start = time.perf_counter()
    with torch.no_grad():
        actions = model(obs, goal)
    torch.cuda.synchronize()
    latencies.append((time.perf_counter() - start) * 1000)

act_window = kwargs["act_window_size"]
avg_ms = sum(latencies) / len(latencies)
avg_hz = act_window / (avg_ms / 1000)

print(f"\n{'='*60}")
print("FlowerVLA Results")
print(f"{'='*60}")
print(f"Model: {MODEL_REPO}")
print(f"Action window size: {act_window}")
print(f"Action dim: {kwargs['action_dim']}")
print(f"Num sampling steps: {kwargs['num_sampling_steps']}")
print(f"Iterations: {num_iterations}")
print(f"Avg latency: {avg_ms:.2f} ms ({avg_ms/1000:.6f} s)")
print(f"Avg Hz: {avg_hz:.2f} Hz")
print(f"Max GPU memory: {torch.cuda.max_memory_allocated() / 1024**2:.2f} MB")
print(f"{'='*60}")
