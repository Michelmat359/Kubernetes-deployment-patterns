# Dockerfile.dynamic-base
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    CUDA_PKG_VERSION=11-8

# 1. Instalamos utilidades básicas y habilitamos universe
RUN apt-get update && apt-get install -y \
      curl gnupg2 lsb-release ca-certificates software-properties-common \
    && add-apt-repository universe

# 2. Añadimos repositorio ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# 3. Instalamos ROS2 base, usb_cam, OpenCV y herramientas de build
RUN apt-get update && apt-get install -y \
      ros-$ROS_DISTRO-ros-base \
      ros-$ROS_DISTRO-usb-cam \
      python3-pip git \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
      libopencv-dev python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 4. Inicializamos rosdep
RUN rosdep init && rosdep update

# 5. Instalamos PyTorch+CUDA y Ultralytics (sin modelo)
RUN pip3 install --no-cache-dir \
      --extra-index-url https://download.pytorch.org/whl/cu118 \
      torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 \
    && pip3 install --no-cache-dir ultralytics typing-extensions>=4.4

# 6. EntryPoint genérico
RUN echo '#!/usr/bin/env bash\n\
set -e\n\
source /opt/ros/'"$ROS_DISTRO"'/setup.bash\n\
exec "$@"' > /ros2_entrypoint.sh \
    && chmod +x /ros2_entrypoint.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["bash"]
