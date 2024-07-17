import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    visual = GroupAction([
         IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_path, 'launch','robot-gazebo.launch.py')
        )),
        # TimerAction(period=20.0,actions=[
            Node(
            package='rviz2',
            executable='rviz2',
            arguments=["-d", os.path.join(pkg_path,"rviz","model.rviz")],
            output='screen'
    )
            # ])
    ])

    return  LaunchDescription([
        visual
    ])