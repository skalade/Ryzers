ARG BASE_IMAGE
FROM ${BASE_IMAGE}

WORKDIR /ryzers

# Suppress prompts in scripts
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
	libboost-filesystem1.74.0 \
	vulkan-tools \
	mesa-vulkan-drivers \
	git-lfs

RUN apt install -y vulkan-tools mesa-vulkan-drivers
RUN add-apt-repository ppa:deadsnakes/ppa && \
	apt update && \
	apt install -y python3.10 python3.10-venv python3.10-dev libboost-filesystem1.74.0

RUN git clone https://github.com/amd/RyzenAI-SW
RUN cd RyzenAI-SW && \
    git lfs install && \
    git lfs pull

COPY fix_symlinks.sh .
RUN cp /ryzers/fix_symlinks.sh /ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library/linux/onnx/ryzen14/
RUN cd /ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library/linux/onnx/ryzen14/ && ./fix_symlinks.sh

RUN cp /ryzers/fix_symlinks.sh /ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library/linux/onnx/cpu/
RUN cd /ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library/linux/onnx/cpu/ && ./fix_symlinks.sh

RUN cd /ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library/samples && \
	curl -o video_call.mp4 https://videos.pexels.com/video-files/8136072/8136072-hd_1920_1080_25fps.mp4 && \
	mkdir build-samples && \
	cmake -S $PWD -B $PWD/build-samples && \
	cmake --build $PWD/build-samples --config Release

COPY ryzen_ai_depth_estimation ./ryzen_ai_depth_estimation

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y ros-kilted-ros-base ros-dev-tools ros-kilted-cv-bridge ros-kilted-image-transport

RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc

RUN pip install catkin_pkg opencv-python
RUN pip install "numpy<2"

ENV LD_LIBRARY_PATH=/ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library/linux/onnx/cpu:/ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library/linux:$LD_LIBRARY_PATH

# Cleanup
ENV SHELL=/bin/bash

COPY test.sh /ryzers/
CMD ["/ryzers/test.sh"]
