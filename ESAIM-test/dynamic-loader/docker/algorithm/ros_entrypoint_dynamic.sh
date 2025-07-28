#!/usr/bin/env bash
set -e

# 1. Descarga dinámica de pesos si no existen
if [ ! -f "$MODEL_PATH" ]; then
  echo "[entrypoint] Descargando pesos del modelo desde $MODEL_URL ..."
  wget -q --show-progress -O "$MODEL_PATH" "$MODEL_URL"
fi

# 2. Sourcing ROS2 y workspace
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros2_ws/install/setup.bash

# 3. Arranca el nodo YOLO
exec "$@"
