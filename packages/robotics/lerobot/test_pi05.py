# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

import torch
import time
from lerobot.policies.pi05.modeling_pi05 import PI05Policy
from lerobot.policies.pi05.configuration_pi05 import PI05Config
from lerobot.policies.pi05.processor_pi05 import make_pi05_pre_post_processors

# Load model
print("Loading Pi0.5 model...")
policy = PI05Policy.from_pretrained("lerobot/pi05_base").to("cuda")
policy.eval()

# Create dataset stats for the processor
# These are dummy stats since we're just testing inference
# Pi0.5 uses quantile normalization, so we need q01 and q99 stats
state_dim = policy.config.max_state_dim
action_dim = policy.config.max_action_dim

# For quantile normalization, we provide q01 (1st percentile) and q99 (99th percentile)
# These will be used to map values to a normalized range
dataset_stats = {
    "observation.state": {
        "q01": torch.full((state_dim,), -1.0),
        "q99": torch.full((state_dim,), 1.0),
    },
    "action": {
        "q01": torch.full((action_dim,), -1.0),
        "q99": torch.full((action_dim,), 1.0),
    },
}

# Create preprocessor and postprocessor
preprocessor, postprocessor = make_pi05_pre_post_processors(
    config=policy.config,
    dataset_stats=dataset_stats
)

print(f"Model loaded successfully!")
print(f"  PaliGemma variant: {policy.config.paligemma_variant}")
print(f"  Action expert variant: {policy.config.action_expert_variant}")
print(f"  Chunk size: {policy.config.chunk_size}")
print(f"  Max state dim: {state_dim}")
print(f"  Max action dim: {action_dim}")
print(f"  Normalization: Quantiles (min-max based)")

# Dummy batch config for a single observation
batch_size = 1
img_shape = (3, 224, 224)  # (C, H, W) - Pi0.5 uses 224x224 images

# Create dummy batch (before preprocessing)
# Note: Pi0.5 encodes state as discretized tokens in the language prompt
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
print(f"  Note: Pi0.5 discretizes state into 256 bins and includes it in the language prompt")

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

print(f"\n=== Pi0.5 Benchmark Results ===")
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
