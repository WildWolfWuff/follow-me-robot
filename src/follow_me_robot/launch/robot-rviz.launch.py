import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'follow_me_robot'
    model_file_path = 'urdf/robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),model_file_path)
    bot_description_compiled = xacro.process_file(xacro_file).toxml()
    
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': bot_description_compiled,
        'use_sim_time': True}] # add other parameters here if required
    )
    
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return  LaunchDescription([
        node_joint_state_publisher,
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2
    ])
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #     )

    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #             arguments=['-topic', 'robot_description',
    #                         '-entity', 'my_bot'],
    #             output='screen')