#!/bin/bash

# Copyright (C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# Fixed paths
export PATH=/ryzers/llamacpp/build/bin/:$PATH
MODEL="ggml-org/gemma-3-4b-it-GGUF"

# Fixed prompt
IMAGE="/ryzers/data/toucan.jpg"
PROMPT="Describe what you see in the image in detail."

# Run
llama-mtmd-cli -hf $MODEL --image $IMAGE -p "$PROMPT"