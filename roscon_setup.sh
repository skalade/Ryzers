#!/bin/bash

git clone https://github.com/AMDResearch/Ryzers
cd Ryzers
pip install -e .

# Setup xdna driver
# For NPU to work secure boot needs to be disabled in BIOS
./packages/npu/xdna/install_xdna.sh

ryzers build genesis roscon25-dt --name roscon-dt
ryzers build llamacpp ros smolvla amdgpu_top roscon25-gpu --name roscon-gpu
ryzers build xdna ryzenai_cvml ros roscon25-npu --name roscon-npu
