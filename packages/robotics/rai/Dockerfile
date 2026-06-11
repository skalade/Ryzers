# Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

ARG BASE_IMAGE=ros
FROM ${BASE_IMAGE}
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

# ROS2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git unzip ffmpeg libportaudio2 python3-venv \
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
RUN pip install --upgrade pip "setuptools<82" poetry "empy<4" lark && \
    pip install --no-build-isolation openai-whisper==20231117

# RAI framework (pinned commit)
WORKDIR /ryzers
RUN git clone https://github.com/RobotecAI/rai.git
WORKDIR /ryzers/rai
RUN git checkout a097617
RUN vcs import < ros_deps.repos
RUN vcs import < demos.repos

RUN poetry config virtualenvs.create false && \
    poetry install --no-interaction

RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src/examples/rai-manipulation-demo/ros2_ws/src --ignore-src -r -y

RUN . ./scripts/download_demo.sh manipulation

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# poetry installs only rai_core + rai_whoami; the demo also needs these extensions.
RUN pip install \
    -e src/rai_sim \
    -e src/rai_bench \
    -e src/rai_extensions/rai_perception

# Pin the base image's ROCm torch/triton wheels (poetry + extensions pull mismatched ones).
ARG ROCM_WHEEL_BASE=https://repo.radeon.com/rocm/manylinux/rocm-rel-7.2.2
RUN pip install --force-reinstall --no-deps \
    ${ROCM_WHEEL_BASE}/torch-2.10.0%2Brocm7.2.2.lw.git40d237bf-cp312-cp312-linux_x86_64.whl \
    ${ROCM_WHEEL_BASE}/torchvision-0.25.0%2Brocm7.2.2.git82df5f59-cp312-cp312-linux_x86_64.whl \
    ${ROCM_WHEEL_BASE}/torchaudio-2.10.0%2Brocm7.2.2.git5047768f-cp312-cp312-linux_x86_64.whl \
    ${ROCM_WHEEL_BASE}/triton-3.6.0%2Brocm7.2.2.git4ed88892-cp312-cp312-linux_x86_64.whl

# Vulkan ICD/driver for the O3DE renderer (base ships only the loader, not a driver).
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-vulkan-drivers libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ryzers
COPY test.sh /ryzers/test_rai.sh
RUN chmod +x /ryzers/test_rai.sh
COPY manipulation_demo.sh /ryzers/manipulation_demo.sh
RUN chmod +x /ryzers/manipulation_demo.sh

EXPOSE 8501
CMD /ryzers/test_rai.sh
