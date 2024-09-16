# follow-me-robot
A robot that follows a point
# Installations

To install the necessary packages, run the following command:

```bash
./setup-ros2.sh
```

This script will install the ros2 humble distribution and the necessary packages.

The install steps are:

- setup git config, enter username and email
- [install ros2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
  1. install dependencies: curl, gnupg2, lsb-release, software-properties-common
  2. set language to en_US.UTF-8
  4. add ros2 apt repository
  5. install ros dev tools
  6. install ros2 ros-base
  7. install ros2 ros-desktop-full
  8. install colcon
- Add ros source to bashrc/zshrc if not exists
- Initialize ros
  1. Init rosdep
  2. Update rosdep
  3, Build projects with colcon
- Create Links for gazebo models
- Install imu-tools and gps-tools for gazebo simulation
- install vscode extensions from recomended extensions file and open vscode

# Run

## Robot simulation

To run the simulation, run `./compile.bash` (Bash shell) or `./compile.zsh` (zsh shell) inside the root directory.
This script install missing dependencies and build the projects.
After the build it launches the [follow-me-robot robot.launch.py](src/follow_me_robot/launch/robot.launch.py) file, in simulation mode. Also it starts the foxglove bridge.

## Configurations

The Node configuration are in the [config](src/follow_me_robot/config) directory. The configuration files are in yaml format.

# Self controled Tag Robot

If you want to run the self controled TagRobot, you can enable the spawn mock with `spawn_mock_bot:=true` in the compile file.
To controll the mock robot, publish the teleopt twist message to the `/mock/cmd_vel` topic. This can be done with the following command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mock/cmd_vel
```
or in foxglove with the `Joystick Panel` plugin.
Also you have to change the tag id configuration (`path_builder:ros__parameters:tag:id`) in the [sensor_config.yaml](src/follow_me_robot/config/sensor_config.yaml) configuration file.

# Refs and links
- 
- [Completed worlds](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps)
- [April Tag ros package](https://github.com/christianrauch/apriltag_ros)
- [Navigation 2](https://docs.nav2.org/index.html)
- [URDF example](https://github.com/joshnewans/urdf_example)
- [ROS2 URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [ROS2 Xacro](http://wiki.ros.org/xacro)
- [ROS2 Gazebo Sensor Plugins (Migraton ROS1 to ROS2)](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)
- [ROS2 Gazebo sensor Tutorials](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
- [ROS2 Gazebo sensor Tutorial Video](https://www.youtube.com/watch?v=laWn7_cj434)
- [SDFormat](http://sdformat.org/spec)
- [April Tag](https://april.eecs.umich.edu/software/apriltag.html)
- [Launch files Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [lidar-xacro](https://github.com/joshnewans/articubot_one/blob/545acac87ae215d80ef6b28abe6097eb7281d9ff/description/lidar.xacro)
- [VS-code intelisens support](https://www.youtube.com/watch?v=hf76VY0a5Fk)
- [Mecanum bot ros pjoect](https://www.youtube.com/watch?v=sb7FoOGzb8E)


https://lcamp.eu