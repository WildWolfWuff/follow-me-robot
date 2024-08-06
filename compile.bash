#!/bin/bash
used_shell="bash" # ${SHELL##*/}
colcon build --symlink-install --event-handlers log_command+
if [ $? -ne 0 ]; then
  echo "Build failed"
  return 1
fi
setup_path="$PWD/install/setup.$used_shell"
echo "impport: $setup_path"
. $setup_path
ros2 launch follow_me_robot robot.launch.py start_sim:=true use_foxglove:=true