# ROS 2 Humble (Ubuntu 22.04)
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42
ENV YOLO_VERBOSE=False

# Dependencias del sistema y ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions \
      python3-opencv libgl1 libglib2.0-0 \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-image-transport \
    && rm -rf /var/lib/apt/lists/*

# PyTorch + CUDA 12.1 + Ultralytics
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
      torch==2.4.1+cu121 torchvision==0.19.1+cu121 \
      --index-url https://download.pytorch.org/whl/cu121 && \
    python3 -m pip install --no-cache-dir ultralytics==8.3.2 numpy

# Workspace
WORKDIR /opt/ros_ws
COPY src/yolo_detector_pkg src/yolo_detector_pkg

# instalar dependencias Python
RUN if [ -f src/yolo_detector_pkg/requirements.txt ]; then \
      python3 -m pip install --no-cache-dir -r src/yolo_detector_pkg/requirements.txt ; \
    fi

SHELL ["/bin/bash", "-lc"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --packages-select yolo_detector_pkg

# Pre-descarga del modelo
RUN python3 - <<'PY'\nfrom ultralytics import YOLO\nYOLO('yolov8n.pt')\nPY

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Lanza el detector
CMD ["ros2", "launch", "yolo_detector_pkg", "yolo_detector.launch.py"]

# ======= CPU (opcional) =======
# Sustituye el bloque "PyTorch + CUDA" por:
# RUN python3 -m pip install --no-cache-dir --upgrade pip && \
#     python3 -m pip install --no-cache-dir \
#       torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cpu && \
#     python3 -m pip install --no-cache-dir ultralytics==8.3.2 numpy
