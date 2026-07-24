# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Flattened ryzer_env -> ros -> o3de -> rai -> lemonade-sdk chain.

FROM rocm/pytorch:rocm7.2.2_ubuntu24.04_py3.12_pytorch_release_2.10.0

# =============================================================================
# ryzer_env: iGPU groups
# =============================================================================
RUN getent group render || groupadd render
RUN getent group video  || groupadd video
RUN getent group kvm    || groupadd kvm
RUN usermod -aG render root && usermod -aG video root && usermod -aG kvm root

# RAI has only been validated against the jazzy release
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

# =============================================================================
# ros: ROS 2 desktop + dev tools
# =============================================================================
# 1. Set locale
RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 2. Enable universe repo
RUN apt-get update && apt-get install -y software-properties-common curl gnupg2 sudo \
 && add-apt-repository universe \
 && rm -rf /var/lib/apt/lists/*

# 3. Add ROS 2 apt repository
RUN apt update && sudo apt install curl -y && \
   export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
   curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
dpkg -i /tmp/ros2-apt-source.deb

# 4. Install ROS 2 (desktop variant)
RUN apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y ros-${ROS_DISTRO}-desktop \
 && rm -rf /var/lib/apt/lists/*

# 5. Install dev tools
RUN apt-get update \
 && apt-get install -y \
    ros-dev-tools \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-web-video-server \
    ros-${ROS_DISTRO}-v4l2-camera \
 && rm -rf /var/lib/apt/lists/*

# 6. Install Python packages
RUN pip install catkin_pkg opencv-python
RUN pip install "numpy<2"

# 7. Source ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# =============================================================================
# o3de: Vulkan + O3DE
# =============================================================================
WORKDIR /ryzers

# Install Vulkan
RUN wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo tee /etc/apt/trusted.gpg.d/lunarg.asc
RUN sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-noble.list https://packages.lunarg.com/vulkan/lunarg-vulkan-noble.list
RUN sudo apt update
RUN sudo apt install -y vulkan-sdk

RUN getent group audio || groupadd audio
RUN usermod -aG audio root

RUN mkdir -p -m 0700 /run/user/0
ENV XDG_RUNTIME_DIR=/run/user/0

# Disable sound in docker
RUN echo 'pcm.!default = null;' >> /root/.asoundrc

# Install O3DE
RUN wget https://o3debinaries.org/main/Latest/Linux/o3de_2510_0.deb
RUN apt install -y ./o3de_2510_0.deb
RUN rm ./o3de_2510_0.deb

# =============================================================================
# rai: Robot Agent Interface framework
# =============================================================================
# ROS2 dependencies. Graphics packages (libgl1-mesa-dri + vulkan) added for the
# O3DE renderer, which the base image doesn't fully provide
RUN apt-get update && apt-get install -y --no-install-recommends \
    git unzip ffmpeg libportaudio2 python3-venv \
    libgl1-mesa-dri libvulkan1 mesa-vulkan-drivers vulkan-tools \
    ros-${ROS_DISTRO}-rai-interfaces \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-sdformat-urdf \
 && rm -rf /var/lib/apt/lists/*

# Rust toolchain (needed by rai_interfaces__rs ROS2 package)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
    | sh -s -- -y --default-toolchain stable --no-modify-path
ENV PATH=/root/.cargo/bin:$PATH

RUN echo "/opt/ros/${ROS_DISTRO}/opt/sdformat_vendor/lib" > /etc/ld.so.conf.d/ros-sdformat.conf && \
    ldconfig

# Activate base image venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN echo "source $VIRTUAL_ENV/bin/activate" >> /root/.bashrc

# Pin empy<4 (required by rosidl_generator_rs)
RUN pip install --upgrade pip "setuptools<82" "empy<4" lark && \
    pip install --no-build-isolation openai-whisper==20231117

# RAI framework (pinned commit)
WORKDIR /ryzers
RUN git clone https://github.com/RobotecAI/rai.git

WORKDIR /ryzers/rai
RUN git checkout e54f8ca
RUN vcs import < ros_deps.repos
RUN vcs import < demos.repos

# TODO: Once upstream fixed remove the grabbing_point_tool patch
# sed fixes an upstream bug: GetObjectPositionsTool's forward-ref to
# GetGrabbingPointTool is never rebuilt, so under langchain 1.x / pydantic 2.x the
# model is "not fully defined" and instantiation raises. Any avoids it.
RUN sed -i 's/    get_grabbing_point_tool: "GetGrabbingPointTool"/    get_grabbing_point_tool: Any/' \
        src/rai_core/rai/tools/ros2/manipulation/custom.py && \
    pip install src/rai_core src/rai_whoami

RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src/examples/rai-manipulation-demo/ros2_ws/src --ignore-src -r -y

RUN . ./scripts/download_demo.sh manipulation

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# rai_bench / rai_sim live in the repo but aren't declared as poetry/colcon
# deps, so install from source; rai_perception comes from PyPI. It must go
# before the torch re-pin below (it would otherwise pull CUDA torch back in)
RUN pip install src/rai_bench && \
    pip install src/rai_sim && \
    pip install rai-perception

# Pre-download detection/segmentation weights (~1.5 GB) via rai-perception's own
# downloader, so model/URL bumps flow through automatically.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . install/setup.sh && \
    download-perception-models

# Reinstall ROCm torch (poetry deps pull CUDA torch)
RUN pip install torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/rocm7.2/ \
    --force-reinstall --no-deps

# Swap the CUDA triton (pulled with torch above) for the ROCm build; the CUDA
# one lacks triton.backends, which ROCm torch imports via inductor
RUN pip uninstall -y triton && \
    pip install pytorch-triton-rocm \
        --index-url https://download.pytorch.org/whl/rocm7.2/ --no-deps

WORKDIR /ryzers

# Individual ryzer sign-of-life tests (ros, o3de, rai, lemonade-sdk)
COPY test_ros.sh test_o3de.sh test_rai.sh test_lemonade-sdk.sh /ryzers/
RUN chmod +x /ryzers/test_*.sh

# =============================================================================
# lemonade-sdk: local OpenAI-compatible model server
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        wget \
        git

# Install lemonade-server from PPA.
RUN add-apt-repository ppa:lemonade-team/stable
RUN apt-get install -y lemonade-server

# Fix: create runtime dir and set XDG_RUNTIME_DIR for lemond
# Workaround for lemonade-sdk#2083 (XDG_RUNTIME_DIR).
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# =============================================================================
# Workshop: JupyterLab + notebooks
# =============================================================================
RUN pip install jupyterlab "numpy<2" matplotlib ipywidgets

COPY notebooks /ryzers/notebooks/
RUN chmod -R 777 /ryzers/notebooks

# Make ROS + RAI environment available to all interactive shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f /ryzers/rai/install/setup.bash ] && source /ryzers/rai/install/setup.bash" >> /etc/bash.bashrc

# JupyterLab entrypoint (sources ROS + RAI before launching)
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'export SHELL=/bin/bash' >> /entrypoint.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /entrypoint.sh && \
    echo '[ -f /ryzers/rai/install/setup.bash ] && source /ryzers/rai/install/setup.bash' >> /entrypoint.sh && \
    echo 'exec python3 -m jupyterlab --ip=0.0.0.0 --port=8888 --allow-root --no-browser "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

WORKDIR /ryzers/notebooks

EXPOSE 8888
EXPOSE 11311
EXPOSE 13305

CMD ["/bin/bash", "/entrypoint.sh"]
