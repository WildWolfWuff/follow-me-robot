import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,InvalidLaunchFileError
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
        # ros2 run apriltag_ros apriltag_node --ros_args -remap image_rect:=/rrbot/cam/front -remap camera_info:=/rrbot/cam/front/camera_info --params-file /home/dev/follow-me-robot/install/follow_me_robot/share/follow_me_robot/config/apriltag.yaml
    # ros2 run apriltag_ros apriltag_node --ros-args -remap image_rect:=/rrbot/cam/front/image_raw -remap camera_info:=/rrbot/cam/front/camera_info --params-file `ros2 pkg prefix follow_me_robot`/share/follow_me_robot/config/apritag.yaml
    april_node=Node(
      package='apriltag_ros',
      executable='apriltag_node',
      remappings=[
        ('image_rect','/rrbot/cam/front/image_raw'),
        ('camera_info','/rrbot/cam/front/camera_info'),
        ('/tf','/tf/tag')
      ],
      parameters=[
        os.path.join(pkg_path,'config','apritag.yaml')
      ]
    )
    return LaunchDescription([
      april_node
    ])