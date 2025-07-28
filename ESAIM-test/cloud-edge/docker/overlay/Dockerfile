FROM esaim:overlay-ros

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

# cámara
RUN apt-get update && apt-get install -y \
      ros-$ROS_DISTRO-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# build overlay
WORKDIR /ros2_ws/src
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git \
 && git clone https://github.com/mgonzs13/yolo_ros.git

COPY launch_yolo.py yolo_ros/yolo_bringup/launch/launch_yolo.py

# PyTorch + Ultralytics aquí
WORKDIR /ros2_ws
RUN pip3 install --no-cache-dir \
      --extra-index-url https://download.pytorch.org/whl/cu118 \
      torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 \
    && pip3 install --no-cache-dir ultralytics typing-extensions>=4.4


# compila solo overlay
RUN /bin/bash -lc "\
      source /opt/ros/$ROS_DISTRO/setup.bash && \
      rosdep install --from-paths src --ignore-src -r -y && \
      colcon build --symlink-install \
    "

# entrypoint
RUN echo '#!/usr/bin/env bash\n\
source /opt/ros/$ROS_DISTRO/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec \"$@\"' > /ros_entrypoint_overlay.sh \
    && chmod +x /ros_entrypoint_overlay.sh

ENTRYPOINT ["/ros_entrypoint_overlay.sh"]
CMD ["ros2", "launch", "yolo_bringup", "launch_yolo.py"]
