import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # define package name
    pkg_name = 'followme_robot_model'
    pkg_path = get_package_share_directory(pkg_name)
    namespace = 'Follow-Me-Robot'
    
    # define and set path to xacro model filerobot_body_footprint
    model_file_path = 'robot/robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),model_file_path)
    
    # compile model
    bot_description_compiled = xacro.process_file(xacro_file).toxml()
    
    # define start script for robot state publisher to publish the compiled robto description
    
    gazebo = GroupAction(
        [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': bot_description_compiled,
            'use_sim_time': True}] # add other parameters here if required
        ),
        TimerAction(period=0.1,actions=[
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']
                )
            ),
            Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'mecanum-bot'],
                    output='screen'),
        ]),
        TimerAction(period=10.0,actions=[
            Node(package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_path,'rviz','model.rviz')],
            output='screen')
        ])
    ])
    # define start script for gazebo ui

    # retuns the defined launch scripts
    ld = LaunchDescription([])
    ld.add_action(gazebo)
    return ld