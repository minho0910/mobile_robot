#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

test "${ROS_DISTRO}" = jazzy
command -v ros2 >/dev/null
command -v colcon >/dev/null
ros2 bag --help >/dev/null
ros2 interface show sensor_msgs/msg/PointCloud2 >/dev/null
ros2 interface show sensor_msgs/msg/Imu >/dev/null
ros2 interface show nav_msgs/msg/Odometry >/dev/null
ros2 interface show rosgraph_msgs/msg/Clock >/dev/null

echo "ROS 2 Jazzy environment and required message types: OK"
