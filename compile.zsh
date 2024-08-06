#!/bin/zsh
used_shell="zsh" # ${SHELL##*/}
colcon build --symlink-install --event-handlers log_command+
if [ $? -ne 0 ]; then
  echo "Build failed"
  return 1
fi
setup_path="$PWD/install/setup.$used_shell"
echo "impport: $setup_path"
. $setup_path

world_path=~/.gazebo/models/hospital/hospital.world
world_models=~/.gazebo/models/hospital/models
ros2 launch follow_me_robot robot.launch.py start_sim:=true use_foxglove:=true world:=$world_path world_models:=$world_models bot_x:=-1 bot_y:=1