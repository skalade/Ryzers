#!/bin/bash
# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

set -e

echo "=========================================="
echo "RAI Framework Test"
echo "=========================================="

# Source ROS environment (ROS_DISTRO set by ros package)
if [ -n "$ROS_DISTRO" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "ROS 2 ${ROS_DISTRO} environment sourced"
else
    echo "Warning: ROS_DISTRO not set. Build with: ryzers build ros rai"
fi

# Test RAI core import
echo ""
echo "Testing RAI core import..."
python3 -c "
from rai.agents import AgentRunner, ReActAgent
from rai.initialization import get_llm_model
print('RAI core imports successful!')
print('  - AgentRunner: available')
print('  - ReActAgent: available')
print('  - get_llm_model: available')
"

# Test RAI whoami import
echo ""
echo "Testing RAI whoami import..."
python3 -c "
from rai_whoami import RobotProfile
print('RAI whoami imports successful!')
print('  - RobotProfile: available')
"

# Check ROS 2 tools
echo ""
echo "Testing ROS 2 integration..."
ros2 --help > /dev/null 2>&1 && echo "ROS 2 CLI: available"

# Print versions
echo ""
echo "=========================================="
echo "Version Information"
echo "=========================================="
python3 -c "import rai; print(f'RAI version: check pyproject.toml')"
echo "ROS 2 distro: ${ROS_DISTRO}"
python3 --version

echo ""
echo "=========================================="
echo "RAI Framework test completed successfully!"
echo "=========================================="
echo ""
echo "To get started with RAI:"
echo "  1. Set your LLM API key (e.g., OPENAI_API_KEY)"
echo "  2. Run: rai-config-init"
echo "  3. See documentation: https://robotecai.github.io/rai/"
