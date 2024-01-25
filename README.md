# follow-me-robot
A rebot that follows a point

# Run

## Robot simulation

run `colcon build` in root director then run `source install/setup.bash`.  
Afert that run `ros2 launch followme_robot_model robot.launch.`

# Refs

- [URDF example](https://github.com/joshnewans/urdf_example)
  - launch files
- zip from mecanum bot
- [lidar-xacro](https://github.com/joshnewans/articubot_one/blob/545acac87ae215d80ef6b28abe6097eb7281d9ff/description/lidar.xacro)
- [VS-code intelisens support](https://www.youtube.com/watch?v=hf76VY0a5Fk)
# TODOs

- [x] add rviz config file
  - [examle](https://github.com/turtlebot/turtlebot4_desktop/blob/humble/turtlebot4_viz/rviz/robot.rviz)

- [ ] Add controller for driving robot, see: [example](https://github.com/DeborggraeveR/ros2-mecanum-bot)