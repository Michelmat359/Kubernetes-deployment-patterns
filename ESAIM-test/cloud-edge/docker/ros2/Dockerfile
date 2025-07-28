FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
      curl gnupg2 lsb-release ca-certificates git wget python3-pip \
      libopencv-dev python3-opencv libboost-all-dev \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         | gpg --dearmor -o /usr/share/keyrings/ros2-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
         ros-$ROS_DISTRO-ros-base \
         ros-$ROS_DISTRO-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# build tools
RUN pip3 install --no-cache-dir \
      colcon-common-extensions rosdep vcstool \
    && rosdep init && rosdep update

# entrypoint
RUN echo '#!/usr/bin/env bash\n\
source /opt/ros/$ROS_DISTRO/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
