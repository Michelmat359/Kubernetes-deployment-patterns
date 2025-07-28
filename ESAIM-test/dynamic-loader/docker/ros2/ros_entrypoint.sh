#!/usr/bin/env bash
set -e

# 1. Sourcing ROS
source /opt/ros/$ROS_DISTRO/setup.bash

# 2. Preparar el workspace
cd $ROS_WS
if [ ! -d src/yolo_ros ]; then
  echo "Clonando repositorios..."
  git clone -b humble https://github.com/ros-perception/vision_opencv.git src/vision_opencv
  git clone        https://github.com/mgonzs13/yolo_ros.git          src/yolo_ros
  cp src/yolo_ros/yolo_bringup/launch/launch_yolo.py src/yolo_ros/yolo_bringup/launch/
fi

# 3. Instalar dependencias ROS
rosdep install --from-paths src --ignore-src -r -y

# 4. Compilar en runtime
colcon build --symlink-install

# 5. Sourcing workspace recién compilado
source install/setup.bash

# 6. Ejecutar comando pedido (p.ej. ros2 launch …)
exec "$@"
