#!/bin/bash
colcon build
source install/setup.bash
ros2 launch followme_robot_model robot-gazebo.launch.py