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
    bot_x=LaunchConfiguration('bot_x')
    bot_y=LaunchConfiguration('bot_y')
    bot_z=LaunchConfiguration('bot_z')
    bot_topic=LaunchConfiguration('bot_topic')
    bot_name=LaunchConfiguration('bot_name')
    declare_world_cmd=DeclareLaunchArgument('world',default_value='',description='world file to load')
    declare_world_models_cmd=DeclareLaunchArgument('world_models',default_value='',description='world file to load')
    declare_bot_x_cmd=DeclareLaunchArgument('bot_x',default_value='0',description='bot x position')
    declare_bot_y_cmd=DeclareLaunchArgument('bot_y',default_value='0',description='bot y position')
    declare_bot_z_cmd=DeclareLaunchArgument('bot_z',default_value='0',description='bot z position')
    declare_bot_topic_cmd=DeclareLaunchArgument('bot_topic',default_value='robot_description',description='robot state publisher topic')
    declare_bot_name_cmd=DeclareLaunchArgument('bot_name',default_value='follow_me_bot',description='robot name')
    # gz_env_path=os.environ.get('GAZEBO_MODEL_PATH',os.path.expanduser('~/.gazebo/models'))
    # export GAZEBO_MODEL_PATH=/home/dev/.gazebo/models/hospital/models
    world_resources=AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',world_models)
    # ros2 launch gazebo_ros gazebo.launch.py world:=/home/dev/.gazebo/models/hospital/hospital.world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose':'true'
            }.items()
        )
    # define start script to spawn the robot from the robot description and name it mecanum-bot
    node_spawn_entity = Node(package='gazebo_ros', 
                             executable='spawn_entity.py',
                    arguments=['-topic', bot_topic,
                                '-entity', bot_name,
                                '-x', bot_x,
                                '-y', bot_y,
                                '-z', bot_z,
                                ],
                    output='screen')
    # retuns the defined launch scripts
    ld= LaunchDescription()
    # arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_models_cmd)
    ld.add_action(declare_bot_x_cmd)
    ld.add_action(declare_bot_y_cmd)
    ld.add_action(declare_bot_z_cmd)
    ld.add_action(declare_bot_topic_cmd)
    ld.add_action(declare_bot_name_cmd)
    # actions
    ld.add_action(world_resources)
    ld.add_action(gazebo)
    ld.add_action(node_spawn_entity)
    return ld