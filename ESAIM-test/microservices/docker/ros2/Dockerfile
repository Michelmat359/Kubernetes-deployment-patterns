# Base Ubuntu 22.04 minimal para ROS 2 Humble
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1. Instalación de ROS 2 base y usb_cam
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release ca-certificates software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
       ros-$ROS_DISTRO-ros-base \
       ros-$ROS_DISTRO-usb-cam \
       python3-colcon-common-extensions python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# 2. Inicializar rosdep
RUN rosdep init && rosdep update

# 3. Copiar launch de cámara y compilar workspace (aunque usb_cam viene por apt)
WORKDIR /ros2_ws/src/usb_cam_bringup
COPY usb_cam_launch.py launch/usb_cam_launch.py

WORKDIR /ros2_ws
RUN /bin/bash -lc " \
      source /opt/ros/$ROS_DISTRO/setup.bash && \
      colcon build --symlink-install \
    "

# 4. Entrypoint
RUN echo "#!/bin/bash\n\
source /opt/ros/$ROS_DISTRO/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec \"\$@\"" \
    > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "usb_cam_bringup", "usb_cam_launch.py"]
