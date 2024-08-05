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
#world_path=~/.gazebo/models/hospital
# world_models=$(ls ~/.gazebo/models/office)
#export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH$world_path/models"
#export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH$world_path"
#ros2 launch gazebo_ros gazebo.launch.py world:=$world_path/hospital.world verbose:=true

ros2 launch follow_me_robot robot.launch.py # start_sim:=true use_foxglove:=true world:=$world_path world_models:=$world_models