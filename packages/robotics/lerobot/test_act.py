# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Test script for LeRobot ACT (Action Chunking Transformer) model
# Based on the structure of test.py for SmolVLA

import torch
import time
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.factory import make_pre_post_processors

# Create model from scratch for testing
print("Creating ACT model...")
# Create a config for ACT with typical ALOHA-like settings
config = ACTConfig(
    chunk_size=50,
    n_action_steps=50,
    n_obs_steps=1,
    vision_backbone="resnet18",
    pretrained_backbone_weights="ResNet18_Weights.IMAGENET1K_V1",
    use_vae=True,
    latent_dim=32,
    n_encoder_layers=4,
    n_decoder_layers=1,
    n_vae_encoder_layers=4,
    dim_model=512,
    n_heads=8,
    dim_feedforward=3200,
    device="cuda",
)

# Set up input and output features
# Typical bimanual ALOHA setup: 2 cameras + proprioceptive state
from lerobot.configs.types import FeatureType, PolicyFeature

config.input_features = {
    "observation.images.cam_high": PolicyFeature(
        type=FeatureType.VISUAL,
        shape=(3, 480, 640),  # Typical camera resolution
    ),
    "observation.images.cam_left_wrist": PolicyFeature(
        type=FeatureType.VISUAL,
        shape=(3, 480, 640),
    ),
    "observation.state": PolicyFeature(
        type=FeatureType.STATE,
        shape=(14,),  # Bimanual: 7 DOF per arm
    ),
}

config.output_features = {
    "action": PolicyFeature(
        type=FeatureType.ACTION,
        shape=(14,),  # Match state dimension for bimanual
    ),
}

# Create the policy
policy = ACTPolicy(config)
policy.to("cuda")
policy.eval()

print(f"Model created successfully!")
print(f"  Vision backbone: {policy.config.vision_backbone}")
print(f"  Chunk size: {policy.config.chunk_size}")
print(f"  n_action_steps: {policy.config.n_action_steps}")
print(f"  n_obs_steps: {policy.config.n_obs_steps}")
print(f"  Use VAE: {policy.config.use_vae}")
print(f"  Latent dim: {policy.config.latent_dim if policy.config.use_vae else 'N/A'}")
print(f"  Dim model: {policy.config.dim_model}")

# Get action and state dimensions from the config
action_feature = policy.config.output_features.get("action")
state_feature = policy.config.input_features.get("observation.state")

if action_feature is None:
    raise ValueError("Action feature not found in config")

action_dim = action_feature.shape[0]

# ACT can work with or without state
if state_feature is not None:
    state_dim = state_feature.shape[0]
    has_state = True
else:
    state_dim = 0
    has_state = False

print(f"  Action dim: {action_dim}")
print(f"  State dim: {state_dim if has_state else 'N/A (vision-only)'}")

# Get image features
image_features = policy.config.image_features
if not image_features:
    raise ValueError("ACT requires at least one image input")

print(f"  Image features: {image_features}")

# Create dataset stats for the processor
# ACT uses mean-std normalization
dataset_stats = {}

# Add image stats for each camera
for img_key in image_features:
    img_feature = policy.config.input_features[img_key]
    img_shape = img_feature.shape
    dataset_stats[img_key] = {
        "mean": torch.zeros(img_shape),
        "std": torch.ones(img_shape),
    }

# Add state stats if present
if has_state:
    dataset_stats["observation.state"] = {
        "mean": torch.zeros(state_dim),
        "std": torch.ones(state_dim),
    }

# Add action stats
dataset_stats["action"] = {
    "mean": torch.zeros(action_dim),
    "std": torch.ones(action_dim),
}

# Create preprocessor and postprocessor
preprocessor, postprocessor = make_pre_post_processors(
    policy_cfg=policy.config,
    dataset_stats=dataset_stats
)

# Dummy batch config for a single observation
batch_size = 1

# Get image shape from first camera
first_img_key = list(image_features.keys())[0]
img_shape = image_features[first_img_key].shape

# Create dummy batch (before preprocessing)
dummy_batch = {}

# Add images for each camera
for img_key in image_features:
    dummy_batch[img_key] = torch.rand(batch_size, *img_shape)

# Add state if present
if has_state:
    dummy_batch["observation.state"] = torch.randn(batch_size, state_dim)

# Add action (needed for training, but we'll only use select_action for inference)
dummy_batch["action"] = torch.randn(batch_size, policy.config.chunk_size, action_dim)

print(f"\nImage shape: {img_shape}")
print(f"Number of cameras: {len(image_features)}")

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

print(f"\n=== ACT Benchmark Results ===")
print(f"Chunk size: {policy.config.chunk_size}")
print(f"n_action_steps: {policy.config.n_action_steps}")
print(f"Avg inference time: {avg_inf:.6f} s")
print(f"Avg inference hz: {avg_hz:.6f} hz")
print(f"Max GPU memory used: {torch.cuda.max_memory_allocated() / 1024**2:.2f} MB")

# Test action prediction
print("\n=== Testing Action Prediction ===")
with torch.no_grad():
    action = policy.select_action(processed_batch)
    print(f"Action shape: {action.shape}")
    print(f"Action (first 5 dims): {action[0, :min(5, action_dim)]}")

print("\n=== Model Architecture ===")
print(f"ACT uses a transformer-based architecture with:")
print(f"  - Vision Encoder: {policy.config.vision_backbone} (pretrained: {policy.config.pretrained_backbone_weights})")
print(f"  - Transformer Encoder: {policy.config.n_encoder_layers} layers")
print(f"  - Transformer Decoder: {policy.config.n_decoder_layers} layers")
if policy.config.use_vae:
    print(f"  - VAE Encoder: {policy.config.n_vae_encoder_layers} layers (latent dim: {policy.config.latent_dim})")
print(f"  - Model dim: {policy.config.dim_model}, Heads: {policy.config.n_heads}")
print(f"  - Feedforward dim: {policy.config.dim_feedforward}")
