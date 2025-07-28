# Dynamic YOLO service sobre CUDA 11.8 + Ubuntu 22.04
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV MODEL_URL=https://myserver/models/yolov8m.pt
ENV MODEL_PATH=/ros2_ws/models/yolov8m.pt

# 1. Instala ROS 2 base, usb_cam, OpenCV, Boost y herramientas de sistema
RUN apt-get update && apt-get install -y \
      software-properties-common \
      curl gnupg2 lsb-release ca-certificates \
      git python3-pip \
      libopencv-dev python3-opencv libboost-all-dev \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
       ros-$ROS_DISTRO-ros-base \
       ros-$ROS_DISTRO-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# 2. Instala colcon, rosdep, vcstool desde pip
RUN pip3 install --no-cache-dir \
      colcon-common-extensions \
      rosdep \
      vcstool

# 3. Inicializa rosdep
RUN rosdep init && rosdep update

# 4. Clona código YOLO y vision_opencv (para cv_bridge)
WORKDIR /ros2_ws/src
RUN git clone https://github.com/mgonzs13/yolo_ros.git \
 && git clone -b humble https://github.com/ros-perception/vision_opencv.git

# 5. Copia tu launch dinámico
COPY launch_yolo.py yolo_ros/yolo_bringup/launch/launch_yolo.py

# 6. Instala PyTorch (CUDA) y Ultralytics
WORKDIR /ros2_ws
RUN pip3 install --no-cache-dir \
      --extra-index-url https://download.pytorch.org/whl/cu118 \
      torch==2.0.1 \
      torchvision==0.15.2 \
      torchaudio==2.0.2 \
    && pip3 install --no-cache-dir \
      ultralytics lap==0.4.0 typing-extensions>=4.4

# 7. Crea carpeta vacía para los pesos
RUN mkdir -p /ros2_ws/models

# 8. Instala dependencias ROS faltantes y compila (saltando cv_bridge apt)
RUN /bin/bash -lc "\
      source /opt/ros/$ROS_DISTRO/setup.bash && \
      rosdep install --from-paths src --ignore-src -r -y --skip-keys=cv_bridge && \
      colcon build --symlink-install \
    "

# 9. Añade script de entrypoint dinámico
COPY ros_entrypoint_dynamic.sh /ros_entrypoint_dynamic.sh
RUN chmod +x /ros_entrypoint_dynamic.sh

ENTRYPOINT ["/ros_entrypoint_dynamic.sh"]
CMD ["ros2", "launch", "yolo_bringup", "launch_yolo.py"]
