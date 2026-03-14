#!/bin/bash

# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

echo "Running tests for lemonade-sdk..."

MODEL="Qwen/Qwen2.5-0.5B-Instruct-GGUF:qwen2.5-0.5b-instruct-q4_0.gguf"

lemonade -i $MODEL llamacpp-load llm-prompt -p "Why is the sky blue?" --max-new-tokens 50
if [ $? -eq 0 ]; then
  echo "Tests passed!"
else
  echo "Tests failed!"
  exit 1
fi
