import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction,IncludeLaunchDescription
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
    sim_launch_file=os.path.join(pkg_path,'launch','gazebo.launch.py')
    # namespace = 'Follow-Me-Robot'
    log_level=LaunchConfiguration('log_level')
    start_sim=LaunchConfiguration('start_sim')
    use_foxglove=LaunchConfiguration('use_foxglove')
    world=LaunchConfiguration('world')
    world_models=LaunchConfiguration('world_models')
    declare_start_sim_cmd=DeclareLaunchArgument('start_sim',default_value='false',description='use simulation')
    declare_use_foxglove_cmd=DeclareLaunchArgument('use_foxglove',default_value='false',description='use foxglove bridge')
    declare_log_level_cmd=DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')
    declare_world_cmd=DeclareLaunchArgument('world',default_value='',description='world file to load')
    declare_world_models_cmd=DeclareLaunchArgument('world_models',default_value='',description='world file to load')
    # define start script for robot state publisher to publish the compiled robto description
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_launch_file),
        launch_arguments={
            "log_level":log_level,
            "use_sim_time":start_sim
            }.items())
    robot_sensors= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_file),
        launch_arguments={
            "log_level":log_level,
            "use_sim_time":start_sim
            }.items())

    sim=GroupAction(
        condition=IfCondition(start_sim),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch_file),
                # set the world file to load
                launch_arguments={
                    "world":world,
                    "world_models":world_models,
                    }.items()
                )
        ]
    )
    foxglove_launch_file=os.path.join(get_package_share_directory("foxglove_bridge"),'launch',"foxglove_bridge_launch.xml")
    start_brige=GroupAction(
        condition=IfCondition(use_foxglove),
        actions=[
            IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_launch_file),
        launch_arguments={"port":"8765"}.items())
        ]
    )
    ld=LaunchDescription()
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_start_sim_cmd)
    ld.add_action(declare_use_foxglove_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_models_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(sim)
    ld.add_action(robot_sensors)
    ld.add_action(start_brige)
    return ld
