#!/bin/bash
used_shell="bash" # ${SHELL##*/}
export ROS_LOG_DIR=./run_logs
rosdep install --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO} -r -y

cd src/follow_me_robot/urdf && xacro robot.urdf.xacro > /dev/null && cd ../../..
if [ $? -ne 0 ]; then
  echo "xacro failed"
  return 1
fi

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
ros2 launch follow_me_robot robot.launch.py start_sim:=true use_foxglove:=true use_gui:=true world:=$world_path world_models:=$world_models bot_x:=-1 bot_y:=1