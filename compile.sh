#!/bin/bash
colcon build --symlink-install
source install/setup.bash
ros2 launch followme_robot_model robot.launch.py