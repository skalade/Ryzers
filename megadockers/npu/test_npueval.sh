#!/bin/bash
# Copyright (C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

source /ryzers/setup.sh
cd /ryzers/npueval
python3 -c "from npueval import dataset, run_functional_tests; run_functional_tests(dataset[:1], results_path='/ryzers/results/canonical')"
