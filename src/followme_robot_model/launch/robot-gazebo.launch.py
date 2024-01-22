import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # define package name
    pkg_name = 'followme_robot_model'
    
    # define and set path to xacro model filerobot_body_footprint
    model_file_path = 'robot/robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),model_file_path)
    
    # compile model
    bot_description_compiled = xacro.process_file(xacro_file).toxml()
    
    # define start script for robot state publisher to publish the compiled robto description
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': bot_description_compiled,
        'use_sim_time': True}] # add other parameters here if required
    )
    
    # define start script for gazebo ui
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
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