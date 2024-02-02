#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash

[ ! -d firmware ] && ros2 run micro_ros_setup create_firmware_ws.sh zephyr discovery_l475_iot1