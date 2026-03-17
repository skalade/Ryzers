#!/bin/bash
# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

set -e

trap 'pkill -9 lemonade-server 2>/dev/null' EXIT

# Pull models and start server
lemonade-server pull Qwen3-VL-4B-Instruct-GGUF 2>&1 | grep -v "Progress:" || true
lemonade-server pull nomic-embed-text-v1-GGUF 2>&1 | grep -v "Progress:" || true
lemonade-server serve --no-tray --global-timeout 600 --ctx-size 8192 > /tmp/lemonade-server.log 2>&1 &
for i in {1..60}; do curl -s http://localhost:8000/health > /dev/null 2>&1 && break; sleep 1; done

# Generate default RAI config then patch it to point at lemonade
cd /ryzers/rai
python -m rai.initialization.config_initialization --force
python3 -c "
import tomli, tomli_w
with open('config.toml', 'rb') as f:
    c = tomli.load(f)
c['vendor'].update(simple_model='openai', complex_model='openai', embeddings_model='openai')
c['openai'].update(simple_model='Qwen3-VL-4B-Instruct-GGUF', complex_model='Qwen3-VL-4B-Instruct-GGUF', embeddings_model='nomic-embed-text-v1-GGUF', base_url='http://localhost:8000/api/v1/')
with open('config.toml', 'wb') as f:
    tomli_w.dump(c, f)
"

export OPENAI_API_KEY="lemonade-local"
source install/setup.bash
source /opt/ros/${ROS_DISTRO}/setup.sh
streamlit run examples/manipulation-demo-streamlit.py
