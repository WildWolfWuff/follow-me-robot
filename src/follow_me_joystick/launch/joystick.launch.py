import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joystick_config = os.path.join(get_package_share_directory('follow_me_joystick'), 'config', 'joy_config.yaml')

    return LaunchDescription([
        Node(
            package='follow_me_joystick',
            executable='follow_me_joystick',
            output='screen',
            parameters=[
                joystick_config
            ]
        )
    ])