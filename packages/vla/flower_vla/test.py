#!/usr/bin/env python3
# Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

import torch
import time
from omegaconf import OmegaConf
from huggingface_hub import hf_hub_download
from safetensors.torch import load_file
from flower.models.flower import FLOWERVLA

MODEL_REPO = "mbreuss/flower_calvin_abc"

# Download model files from HuggingFace
print("\nDownloading FlowerVLA model files...")
config_path = hf_hub_download(MODEL_REPO, "config.yaml")
weights_path = hf_hub_download(MODEL_REPO, "model.safetensors")

# Load config and extract model section
full_cfg = OmegaConf.load(config_path)
model_cfg = full_cfg.model

# Instantiate model directly (~1B params, Florence-2-large backbone + DiT)
print("Instantiating FlowerVLA model...")
model = FLOWERVLA(
    vlm_path=model_cfg.vlm_path,
    freeze_florence=True,
    freeze_vision_tower=True,
    vlm_prompt_style=model_cfg.vlm_prompt_style,
    token_dropout=0.0,  # no dropout for inference
    multistep=model_cfg.multistep,
    num_sampling_steps=model_cfg.num_sampling_steps,
    lowdim_obs_dim=model_cfg.lowdim_obs_dim,
    action_dim=model_cfg.action_dim,
    act_window_size=model_cfg.act_window_size,
    load_pretrained=True,
    pretrained_model_path=weights_path,
    use_second_view=model_cfg.use_second_view,
    second_view_key=model_cfg.second_view_key,
    action_type_adaln=model_cfg.action_type_adaln,
    use_causal_attention=model_cfg.use_causal_attention,
    use_cross_attn=model_cfg.use_cross_attn,
    use_adaln_cond=model_cfg.use_adaln_cond,
    use_readout_token=model_cfg.use_readout_token,
    use_proprio=model_cfg.use_proprio,
    return_act_chunk=model_cfg.get("return_act_chunk", False),
    sampling_type=model_cfg.get("sampling_type", "uniform"),
    dit_dim=model_cfg.dit_dim,
    n_heads=model_cfg.n_heads,
    n_layers=model_cfg.n_layers,
    attn_pdrop=0.0,
    resid_pdrop=0.0,
    mlp_pdrop=0.0,
    use_rope=model_cfg.use_rope,
    use_nope=model_cfg.get("use_nope", False),
    query_seq_len=model_cfg.query_seq_len,
    rope_theta=model_cfg.get("rope_theta", 32.0),
    optimizer_type="adamw",
    optimizer=model_cfg.optimizer,
    lr_scheduler=model_cfg.lr_scheduler,
)
model = model.to("cuda").eval()

# Create dummy inputs matching model's expected format
# rgb images: [batch, time_steps, channels, height, width]
batch_size = 1
img_size = 224  # Florence-2 input resolution
dummy_img = torch.rand(batch_size, 1, 3, img_size, img_size, device="cuda")

obs = {
    "rgb_obs": {
        "rgb_static": dummy_img,
        "rgb_gripper": dummy_img,  # second view (wrist camera)
    },
}
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
    elapsed = time.perf_counter() - start
    latencies.append(elapsed * 1000)  # Convert to ms

act_window_size = model_cfg.act_window_size
avg_latency_ms = sum(latencies) / len(latencies)
avg_latency_s = avg_latency_ms / 1000
avg_hz = act_window_size / avg_latency_s

print(f"\n{'='*60}")
print("FlowerVLA Results")
print(f"{'='*60}")
print(f"Model: {MODEL_REPO}")
print(f"Action window size: {act_window_size}")
print(f"Action dim: {model_cfg.action_dim}")
print(f"Num sampling steps: {model_cfg.num_sampling_steps}")
print(f"Iterations: {num_iterations}")
print(f"Avg latency: {avg_latency_ms:.2f} ms ({avg_latency_s:.6f} s)")
print(f"Avg Hz: {avg_hz:.2f} Hz")
print(f"Max GPU memory: {torch.cuda.max_memory_allocated() / 1024**2:.2f} MB")
print(f"{'='*60}")
