#!/bin/bash
colcon build --symlink-install --event-handlers log_command+
source install/setup.bash
ros2 launch follow_me_robot robot.launch.py