import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction,IncludeLaunchDescription, TimerAction
from launch import LaunchDescription,InvalidLaunchFileError
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
    # namespace = 'Follow-Me-Robot'
    use_foxglove_bridge=LaunchConfiguration('use_foxglove_bridge')
    declare_use_foxglove_bridge_cmd=DeclareLaunchArgument('use_foxglove_bridge',default_value='false',description='use foxglove bridge')
    # define start script for robot state publisher to publish the compiled robto description
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_path, 'launch', 'state.launch.py'
        )))
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_path, 'launch', 'gazebo.launch.py'
        )))
    robot_sensors= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_path, 'launch', 'sensor.launch.py'
        )))
    start_brige=GroupAction(
        condition=IfCondition(use_foxglove_bridge),
        actions=[
            IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory("foxglove_bridge"),'launch',"foxglove_bridge_launch.xml")),
        launch_arguments={"port":"8765"}.items())
        ]
    )
    ld=LaunchDescription()
    ld.add_action(declare_use_foxglove_bridge_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(sim)
    ld.add_action(robot_sensors)
    ld.add_action(start_brige)
    return ld
