used_shell=${SHELL##*/}
echo "Used shell: $used_shell"
colcon build --symlink-install --event-handlers log_command+
source ./install/setup.${used_shell}
ros2 launch follow_me_robot robot.launch.py