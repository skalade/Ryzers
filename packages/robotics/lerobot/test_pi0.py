# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Test script for LeRobot Pi0 model
# Based on the structure of test.py for SmolVLA

import torch
import time
from lerobot.policies.pi0.modeling_pi0 import PI0Policy
from lerobot.policies.pi0.configuration_pi0 import PI0Config
from lerobot.policies.pi0.processor_pi0 import make_pi0_pre_post_processors

# Load model
print("Loading Pi0 model...")
policy = PI0Policy.from_pretrained("lerobot/pi0_base").to("cuda")
policy.eval()

# Create dataset stats for the processor
# These are dummy stats since we're just testing inference
state_dim = policy.config.max_state_dim
action_dim = policy.config.max_action_dim

dataset_stats = {
    "observation.state": {
        "mean": torch.zeros(state_dim),
        "std": torch.ones(state_dim),
    },
    "action": {
        "mean": torch.zeros(action_dim),
        "std": torch.ones(action_dim),
    },
}

# Create preprocessor and postprocessor
preprocessor, postprocessor = make_pi0_pre_post_processors(
    config=policy.config,
    dataset_stats=dataset_stats
)

print(f"Model loaded successfully!")
print(f"  PaliGemma variant: {policy.config.paligemma_variant}")
print(f"  Action expert variant: {policy.config.action_expert_variant}")
print(f"  Chunk size: {policy.config.chunk_size}")
print(f"  Max state dim: {state_dim}")
print(f"  Max action dim: {action_dim}")

# Dummy batch config for a single observation
batch_size = 1
img_shape = (3, 224, 224)  # (C, H, W) - Pi0 uses 224x224 images

# Create dummy batch (before preprocessing)
dummy_batch = {
    # Single image observation
    "observation.images.base_0_rgb": torch.rand(batch_size, *img_shape),
    # Single state observation
    "observation.state": torch.rand(batch_size, state_dim),
    # Task description
    "task": ["pick up the red block"],
    # Action (needed for training, but we'll only use select_action for inference)
    "action": torch.rand(batch_size, policy.config.chunk_size, action_dim),
}

# Preprocess the batch
print("\nPreprocessing batch...")
processed_batch = preprocessor(dummy_batch)

# Move to CUDA
for key in processed_batch:
    if isinstance(processed_batch[key], torch.Tensor):
        processed_batch[key] = processed_batch[key].to("cuda")

print("Batch prepared successfully!")

# Warmup
print("\nWarming up...")
for _ in range(3):
    with torch.no_grad():
        _ = policy.select_action(processed_batch)

# Benchmark
print("\nBenchmarking...")
torch.cuda.reset_peak_memory_stats()
start = time.time()
num_iterations = 100
for _ in range(num_iterations):
    with torch.no_grad():
        action = policy.select_action(processed_batch)
end = time.time()

avg_inf = (end - start) / num_iterations
avg_hz = policy.config.chunk_size / avg_inf

print(f"\n=== Pi0 Benchmark Results ===")
print(f"Chunk size: {policy.config.chunk_size}")
print(f"Avg inference time: {avg_inf:.6f} s")
print(f"Avg inference hz: {avg_hz:.6f} hz")
print(f"Max GPU memory used: {torch.cuda.max_memory_allocated() / 1024**2:.2f} MB")

# Test action prediction
print("\n=== Testing Action Prediction ===")
with torch.no_grad():
    action = policy.select_action(processed_batch)
    print(f"Action shape: {action.shape}")
    print(f"Action (first 5 dims): {action[0, :5]}")
