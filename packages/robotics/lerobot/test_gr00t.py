# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

import torch
import time
from lerobot.policies.groot.modeling_groot import GrootPolicy
from lerobot.policies.groot.configuration_groot import GrootConfig
from lerobot.policies.groot.processor_groot import make_groot_pre_post_processors
from lerobot.configs.types import FeatureType, PolicyFeature

# Load model
print("Loading GR00T model...")
print("  Note: This will download the GR00T model and Eagle processor assets")

# Create config first
config = GrootConfig(
    base_model_path="nvidia/GR00T-N1.5-3B",
    max_state_dim=64,
    max_action_dim=32,
    chunk_size=16,  # GR00T uses action_horizon=16
    n_action_steps=16,
    tune_llm=False,
    tune_visual=False,
    tune_projector=False,
    tune_diffusion_model=False,
    use_bf16=True,
)

# Set up input and output features
config.input_features = {
    "observation.state": PolicyFeature(
        type=FeatureType.STATE,
        shape=(config.max_state_dim,),
    ),
    "observation.images.ego_view": PolicyFeature(
        type=FeatureType.VISUAL,
        shape=(3, 224, 224),
    ),
}

config.output_features = {
    "action": PolicyFeature(
        type=FeatureType.ACTION,
        shape=(config.max_action_dim,),
    ),
}

# Create the policy
policy = GrootPolicy(config)
policy.to("cuda")
policy.eval()

# Create preprocessor and postprocessor
# Note: GR00T preprocessing does include min-max normalization to [-1, 1]
# For testing, we can pass dataset_stats=None to skip normalization
# or provide proper min/max stats if needed
state_dim = config.max_state_dim
action_dim = config.max_action_dim

# Option 1: No normalization (for testing with random data) - RECOMMENDED for testing
dataset_stats = None

# Option 2: Provide min-max stats for normalization (uncomment if needed for real data)
# dataset_stats = {
#     "observation.state": {
#         "min": torch.full((state_dim,), -1.0),
#         "max": torch.full((state_dim,), 1.0),
#     },
#     "action": {
#         "min": torch.full((action_dim,), -1.0),
#         "max": torch.full((action_dim,), 1.0),
#     },
# }

preprocessor, postprocessor = make_groot_pre_post_processors(
    config=config,
    dataset_stats=dataset_stats
)

print(f"Model loaded successfully!")
print(f"  Base model: {config.base_model_path}")
print(f"  Chunk size (action horizon): {config.chunk_size}")
print(f"  Max state dim: {state_dim}")
print(f"  Max action dim: {action_dim}")
print(f"  Normalization: {'Enabled (Min-Max)' if dataset_stats else 'Disabled (for testing)'}")
print(f"  Using bfloat16: {config.use_bf16}")

# Dummy batch config for a single observation
batch_size = 1
img_shape = (3, 224, 224)  # (C, H, W) - GR00T uses 224x224 images by default

# Create dummy batch (before preprocessing)
# Important: Use ego_view as the camera name (matches the test format)
# GR00T expects images in a specific format for Eagle VLM encoding
dummy_batch = {
    # Single image observation (GR00T can handle multiple cameras)
    # Note: Use ego_view as it's a standard camera name
    "observation.images.ego_view": torch.rand(batch_size, *img_shape),
    # Single state observation (use randn for more realistic data)
    "observation.state": torch.randn(batch_size, state_dim) * 0.1,  # Small random values
    # Task description
    "task": ["pick up the red block"],
    # Action (needed for training, but we'll only use predict_action_chunk for inference)
    "action": torch.randn(batch_size, config.chunk_size, action_dim) * 0.1,  # Small random values
}

# Preprocess the batch
print("\nPreprocessing batch...")
print("  Note: GR00T preprocessing includes Eagle VLM encoding of images+language")
processed_batch = preprocessor(dummy_batch)

# Move to CUDA (preprocessor should already do this, but ensure it)
for key in processed_batch:
    if isinstance(processed_batch[key], torch.Tensor):
        processed_batch[key] = processed_batch[key].to("cuda")

print("Batch prepared successfully!")

# Warmup
print("\nWarming up...")
for i in range(3):
    print(f"  Warmup iteration {i+1}/3...")
    with torch.no_grad():
        _ = policy.predict_action_chunk(processed_batch)

# Benchmark
print("\nBenchmarking...")
torch.cuda.reset_peak_memory_stats()
start = time.time()
num_iterations = 100
for i in range(num_iterations):
    if (i + 1) % 10 == 0:
        print(f"  Iteration {i+1}/{num_iterations}")
    with torch.no_grad():
        action = policy.predict_action_chunk(processed_batch)
end = time.time()

avg_inf = (end - start) / num_iterations
avg_hz = config.chunk_size / avg_inf

print(f"\n=== GR00T Benchmark Results ===")
print(f"Chunk size (action horizon): {config.chunk_size}")
print(f"Avg inference time: {avg_inf:.6f} s")
print(f"Avg inference hz: {avg_hz:.6f} hz")
print(f"Max GPU memory used: {torch.cuda.max_memory_allocated() / 1024**2:.2f} MB")

# Test action prediction
print("\n=== Testing Action Prediction ===")
with torch.no_grad():
    action = policy.predict_action_chunk(processed_batch)
    print(f"Action shape: {action.shape}")
    print(f"Action (first 5 dims): {action[0, :5]}")

