#!/bin/bash

# Copyright (C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

amdgpu_top --dump | head

echo "amdgpu_top test completed."
exit 0
