# yolo/Dockerfile
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1. Instalar SO, OpenCV, Boost y añadir repo de ROS 2 Humble
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y \
      curl gnupg2 lsb-release ca-certificates git python3-pip \
      libopencv-dev python3-opencv libboost-all-dev && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y ros-$ROS_DISTRO-ros-base && \
    rm -rf /var/lib/apt/lists/*

# 2. Instalar colcon, rosdep y vcstool vía pip
RUN pip3 install --no-cache-dir \
      colcon-common-extensions \
      rosdep \
      vcstool

# 3. Inicializar rosdep
RUN rosdep init && rosdep update

# 4. Clonar tu paquete YOLO y vision_opencv (para cv_bridge)
WORKDIR /ros2_ws/src
RUN git clone https://github.com/mgonzs13/yolo_ros.git \
 && git clone -b humble https://github.com/ros-perception/vision_opencv.git

# 5. Copiar tu launch personalizado
COPY launch_yolo.py yolo_ros/yolo_bringup/launch/launch_yolo.py

# 6. Instalar PyTorch (CUDA) y Ultralytics YOLOv8
WORKDIR /ros2_ws
RUN pip3 install --no-cache-dir \
      --extra-index-url https://download.pytorch.org/whl/cu118 \
      torch==2.0.1 \
      torchvision==0.15.2 \
      torchaudio==2.0.2 && \
    pip3 install --no-cache-dir ultralytics lap==0.4.0 typing-extensions>=4.4

# 7. Resolver dependencias y compilar todo
RUN /bin/bash -lc "\
      source /opt/ros/$ROS_DISTRO/setup.bash && \
      rosdep install --from-paths src --ignore-src -r -y --skip-keys=cv_bridge && \
      colcon build --symlink-install \
    "

# 8. Entrypoint
RUN echo '#!/bin/bash\n\
source /opt/ros/$ROS_DISTRO/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec \"$@\"' \
 > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "yolo_bringup", "launch_yolo.py"]
