#!/bin/bash
colcon build --event-handlers log_command+
source install/setup.bash
ros2 launch followme_robot_model robot.launch.py -d