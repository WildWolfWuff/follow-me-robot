import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    path_builder_config = os.path.join(get_package_share_directory('followme_path_builder'), 'config', 'path_builder_config.yaml')

    return LaunchDescription([
        Node(
            package='followme_path_builder',
            executable='path_builder',
            output='screen',
            parameters=[
                path_builder_config
            ]
        )
    ])