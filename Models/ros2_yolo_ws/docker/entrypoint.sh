#!/usr/bin/env bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /opt/ros_ws/install/setup.bash ]; then
  source /opt/ros_ws/install/setup.bash
fi
exec "$@"
