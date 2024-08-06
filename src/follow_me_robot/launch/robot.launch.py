import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction,IncludeLaunchDescription,AppendEnvironmentVariable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

# launch the robot state publisher
def generate_launch_description():
    # define package name
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    state_launch_file=os.path.join(pkg_path,'launch','state.launch.py')
    sensor_launch_file=os.path.join(pkg_path,'launch','sensor.launch.py')
    nav_launch_file=os.path.join(pkg_path,'launch','nav.launch.py')
    
    
    # namespace = 'Follow-Me-Robot'
    log_level=LaunchConfiguration('log_level',default='info')
    start_sim=LaunchConfiguration('start_sim',default='false')
    use_foxglove=LaunchConfiguration('use_foxglove',default='false')
    use_rviz=LaunchConfiguration('use_rviz',default='false')

    # define start script for robot state publisher to publish the compiled robto description

    ld=LaunchDescription([
        DeclareLaunchArgument('log_level',default_value=log_level,description='log level'),
        DeclareLaunchArgument('start_sim',default_value='false',description='use simulation'),
        DeclareLaunchArgument('use_foxglove',default_value=use_foxglove,description='use foxglove bridge'),
        DeclareLaunchArgument('use_rviz',default_value=use_rviz,description='use rviz2 for visualization'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(state_launch_file),
            launch_arguments={
                "log_level":log_level,
                "use_sim_time":start_sim
                }.items()),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_file),
        launch_arguments={
            "log_level":log_level,
            "use_sim_time":start_sim
            }.items())
    ])

    # Lauch gazebo simulation if start_sim is true
    verbose_output=LaunchConfiguration('verbose',default='false')
    world=LaunchConfiguration('world',default='')
    world_models=LaunchConfiguration('world_models',default='')
    bot_x=LaunchConfiguration('bot_x',default='0')
    bot_y=LaunchConfiguration('bot_y',default='0')
    bot_z=LaunchConfiguration('bot_z',default='0')
    bot_topic=LaunchConfiguration('bot_topic',default='robot_description')
    bot_name=LaunchConfiguration('bot_name',default='follow_me_bot')
    gazebo_launch_file=os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')
    ld.add_action(GroupAction(
        condition=IfCondition(start_sim),
        actions=[
            DeclareLaunchArgument('verbose',default_value=verbose_output,description='verbose output'),
            DeclareLaunchArgument('world',default_value=world ,description='world file to load'),
            DeclareLaunchArgument('world_models',default_value=world_models ,description='world file to load'),
            DeclareLaunchArgument('bot_x',default_value=bot_x,description='bot x position'),
            DeclareLaunchArgument('bot_y',default_value=bot_y,description='bot y position'),
            DeclareLaunchArgument('bot_z',default_value=bot_z,description='bot z position'),
            DeclareLaunchArgument('bot_topic',default_value=bot_topic,description='robot state publisher topic'),
            DeclareLaunchArgument('bot_name',default_value=bot_name, description='robot name'),
            AppendEnvironmentVariable('GAZEBO_MODEL_PATH',world_models),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file),
                launch_arguments={
                    'world': world,
                    'verbose': verbose_output
                }.items()),
            Node(package='gazebo_ros', 
                executable='spawn_entity.py',
                output='screen',
                arguments=['-topic', bot_topic,
                            '-entity', bot_name,
                            '-x', bot_x,
                            '-y', bot_y,
                            '-z', bot_z,
                            ])
            ]))

    # Launch foxglove bridge if use_foxglove is true
    foxglove_launch_file=os.path.join(get_package_share_directory("foxglove_bridge"),'launch',"foxglove_bridge_launch.xml")
    ld.add_action(GroupAction(
        condition=IfCondition(use_foxglove),
        actions=[IncludeLaunchDescription(
                XMLLaunchDescriptionSource(foxglove_launch_file),
                launch_arguments={
                    "use_sim_time": start_sim,
                    "port":"8765"
                    }.items()
                )]))

    # Launch rviz2 if use_rviz is true
    # TODO: configure rviz2 for current sensors
    rviz_config_file=LaunchConfiguration('rviz_config_file',default=os.path.join(pkg_path,"rviz","model.rviz"))
    ld.add_action(GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            DeclareLaunchArgument('rviz_config_file',default_value=rviz_config_file,description='rviz config file'),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=["-d", rviz_config_file],
                output='screen')
            ]))
    
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(nav_launch_file),
    #     launch_arguments={
    #         "use_sim_time":start_sim
    #         }.items()
    #     ))
    return ld
