#!/bin/bash

# Copyright (C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

cd ncnn/benchmark
../build/benchmark/benchncnn 10 $(nproc) 0 0
