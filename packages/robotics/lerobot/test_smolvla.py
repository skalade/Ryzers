# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Test script for LeRobot SmolVLA model
# Original script from: https://learnopencv.com/smolvla-lerobot-vision-language-action-model/

import torch
import time
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.policies.factory import make_pre_post_processors

# Load model
print("Loading SmolVLA model...")

# Load pretrained SmolVLA model
model_id = "lerobot/smolvla_base"
policy = SmolVLAPolicy.from_pretrained(model_id).to("cuda")
policy.eval()

print(f"Model loaded successfully!")
print(f"  Chunk size: {policy.config.chunk_size}")

# Get state and action dimensions from config
state_feature = policy.config.input_features.get("observation.state")
action_feature = policy.config.output_features.get("action")
state_dim = state_feature.shape[0] if state_feature else 0
action_dim = action_feature.shape[0] if action_feature else 0

print(f"  State dim: {state_dim}")
print(f"  Action dim: {action_dim}")

# Get image features
image_features = policy.config.image_features
if not image_features:
    raise ValueError("SmolVLA requires at least one image input")

# Get first image shape
first_img_key = list(image_features.keys())[0]
img_shape = image_features[first_img_key].shape

print(f"\nImage shape: {img_shape}")
print(f"Number of cameras: {len(image_features)}")

# Dummy batch config for a single observation
batch_size = 1

# Create dummy batch (before preprocessing)
dummy_batch = {}

# Add images for each camera
for img_key in image_features.keys():
    dummy_batch[img_key] = torch.rand(batch_size, *img_shape)

# Add state
dummy_batch["observation.state"] = torch.randn(batch_size, state_dim)

# Add task
dummy_batch["task"] = ["stack the blocks"]

# Add action (needed for training)
dummy_batch["action"] = torch.randn(batch_size, policy.config.chunk_size, action_dim)

print("\nPreprocessing batch...")
# Create preprocessor and postprocessor
preprocessor, postprocessor = make_pre_post_processors(
    policy_cfg=policy.config,
    pretrained_path=model_id,
)

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
        _ = policy.predict_action_chunk(processed_batch)

# Benchmark
print("\nBenchmarking...")
torch.cuda.reset_peak_memory_stats()
start = time.time()
num_iterations = 100
for _ in range(num_iterations):
    with torch.no_grad():
        _ = policy.predict_action_chunk(processed_batch)
end = time.time()

avg_inf = (end - start) / num_iterations
avg_hz = policy.config.chunk_size / avg_inf

print(f"\n=== SmolVLA Benchmark Results ===")
print(f"Chunk size: {policy.config.chunk_size}")
print(f"Avg inference time: {avg_inf:.6f} s")
print(f"Avg inference hz: {avg_hz:.6f} hz")
print(f"Max GPU memory used: {torch.cuda.max_memory_allocated() / 1024**2:.2f} MB")

# Test action prediction
print("\n=== Testing Action Prediction ===")
with torch.no_grad():
    action = policy.predict_action_chunk(processed_batch)
    print(f"Action shape: {action.shape}")
    print(f"Action (first 5 dims): {action[0, :min(5, action_dim)]}")

