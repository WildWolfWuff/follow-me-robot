#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir ~/uros_ws
cd ~/uros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

echo "source ~/uros_ws/install/local_setup.bash" >> ~/.bashrc
source ~/uros_ws/install/local_setup.bash