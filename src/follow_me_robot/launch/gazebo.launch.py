import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # define package name
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    world=LaunchConfiguration('world')
    world_models=LaunchConfiguration('world_models')
    declare_world_cmd=DeclareLaunchArgument('world',default_value='',description='world file to load')
    declare_world_models_cmd=DeclareLaunchArgument('world_models',default_value='',description='world file to load')
    gz_modles_path=os.path.expanduser("~/.gazebo/models")
    # define start script for gazebo ui
    # world_path=os.path.join(gz_modles_path,str(world))
    # export GAZEBO_MODEL_PATH=/home/dev/.gazebo/models/hospital/models
    world_resources=AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',[gz_modles_path,world,"models"])
    )
    # ros2 launch gazebo_ros gazebo.launch.py world:=/home/dev/.gazebo/models/hospital/hospital.world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(world_path,f"{world}.world"),'verbose':'true'}.items()
        )
    # define start script to spawn the robot from the robot description and name it mecanum-bot
    node_spawn_entity = Node(package='gazebo_ros', 
                             executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'mecanum-bot',
                                '-x', '0',
                                '-y', '1',
                                '-z', '0',
                                ],
                    output='screen')
    # retuns the defined launch scripts
    ld= LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(world_resources)
    ld.add_action(gazebo)
    ld.add_action(node_spawn_entity)
    return ld