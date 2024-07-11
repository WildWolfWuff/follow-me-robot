import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, InvalidLaunchFileError
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # define package name
    pkg_name = 'follow_me_robot'
    
    # define and set path to xacro model filerobot_body_footprint
    model_file_path = 'urdf/robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),model_file_path)
    
    # compile model
    bot_description_compiled = xacro.process_file(xacro_file).toxml()
    if bot_description_compiled is None:
        raise InvalidLaunchFileError()

    foxglove_ws= IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory("foxglove_bridge"),'launch',"foxglove_bridge_launch.xml")),
        launch_arguments=[("port","8765")])
   
    # define start script to spawn the robot from the robot description and name it mecanum-bot
    gazebo = GroupAction(
        [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': bot_description_compiled,
            'use_sim_time': True
            }] # add other parameters here if required
        ),
        TimerAction(period=0.5,actions=[
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']
                )
            ),
            Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'mecanum-bot'],
                    output='screen'),
        ])])
    # retuns the defined launch scripts
    ld=LaunchDescription([])
    ld.add_action(foxglove_ws)
    ld.add_action(gazebo)
    return ld