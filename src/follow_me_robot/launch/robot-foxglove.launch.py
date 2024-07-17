import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    # define package name
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    # define and set path to xacro model filerobot_body_footprint    

    foxglove_brige= IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory("foxglove_bridge"),'launch',"foxglove_bridge_launch.xml")),
        launch_arguments=[("port","8765")])
   
    # define start script to spawn the robot from the robot description and name it mecanum-bot
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path,'launch','robot-gazebo.launch.py'
            ))
    )
    # retuns the defined launch scripts
    return LaunchDescription([
        foxglove_brige,
        gazebo
    ])