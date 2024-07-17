import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # define package name
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    
    # define start script for robot state publisher to publish the compiled robto description
    node_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_path, 'launch', 'robot.launch.py'
        )))
    # define start script for gazebo ui

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
        )

    # define start script to spawn the robot from the robot description and name it mecanum-bot
    node_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'mecanum-bot'],
                    output='screen')

    # retuns the defined launch scripts
    return  LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        node_spawn_entity,
    ])