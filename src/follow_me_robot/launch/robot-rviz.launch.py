import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    robot_state_publisher = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
            pkg_path, 'launch', 'robot.launch.py')
        ))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', '/gazebo.launch.py')
        ))
    
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return  LaunchDescription([
        robot_state_publisher,
        gazebo,
        node_rviz2
    ])