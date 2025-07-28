# Base Ubuntu 22.04 con CUDA 11.8 (soporte NVIDIA GPU)
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble


# 1. Instalar ROS 2 Humble y dependencias del sistema
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release ca-certificates software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
          > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-usb-cam \
    git build-essential \
    python3-pip python3-opencv \
    python3-colcon-common-extensions python3-rosdep python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Inicializar rosdep (para resolver dependencias de ROS 2)
RUN rosdep init && rosdep update

# 2. Preparar el espacio de trabajo ROS 2 y clonar el paquete YOLOv8
WORKDIR /ros2_ws
RUN git clone https://github.com/mgonzs13/yolo_ros.git src/yolo_ros

# Copiar el archivo de lanzamiento personalizado al paquete de lanzamiento
COPY monolithic_launch.py src/yolo_ros/yolo_bringup/launch/monolithic_launch.py

# 3. Instalar PyTorch (CUDA) y Ultralytics YOLOv8 via pip
RUN pip3 install --no-cache-dir \
      --extra-index-url https://download.pytorch.org/whl/cu118 \
      torch==2.0.1 \
      torchvision==0.15.2 \
      torchaudio==2.0.2 && \
    pip3 install --no-cache-dir ultralytics lap==0.4.0 typing-extensions>=4.4

# 4. Instalar dependencias ROS faltantes y compilar el workspace
RUN /bin/bash -lc " \
      source /opt/ros/$ROS_DISTRO/setup.bash && \
      rosdep install --from-paths src --ignore-src -r -y && \
      colcon build --symlink-install \
    "



# 5. Configurar la entrada (entrypoint) para el entorno ROS
RUN echo "#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /ros2_ws/install/setup.bash\nexec \"\$@\"" \
      > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# 6. Entrypoint y comando por defecto: lanzar los nodos automáticamente
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "yolo_bringup", "monolithic_launch.py"]
