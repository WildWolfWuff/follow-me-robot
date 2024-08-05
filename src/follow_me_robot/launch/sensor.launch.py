import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,InvalidLaunchFileError
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    log_level=LaunchConfiguration('log_level')
    declare_log_level_cmd=DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')
    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd=DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use sim time')
    # ros2 run follow_me_teleop follow_me_teleop --ros-args --params-file `ros2 pkg prefix follow_me_teleop`/share/follow_me_teleop/config/teleop_config.yaml
    node_teleopt = Node(
        package='follow_me_teleop',
        executable='follow_me_teleop',
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[os.path.join(pkg_path, 'config', 'teleop_config.yaml')],
    )
    # ros2 run apriltag_ros apriltag_node --ros-args -remap image_rect:=/rrbot/cam/front/image_raw -remap camera_info:=/rrbot/cam/front/camera_info --params-file `ros2 pkg prefix follow_me_robot`/share/follow_me_robot/config/apritag.yaml
    april_node=Node(
      package='apriltag_ros',
      executable='apriltag_node',
      remappings=[
        ('image_rect','/sensor/cam/front/image_raw'),
        ('camera_info','/sensor/cam/front/camera_info'),
        ('/tf','/tag/tf'),
        ('/detections','/tag/detections'),
        ('/detections_image','/tag/detections_image')
      ],
      arguments=["--ros-args", "--log-level", log_level],
      parameters=[
        os.path.join(pkg_path,'config','apritag.yaml')]
    )
    return LaunchDescription([
      declare_log_level_cmd,
        declare_use_sim_time_cmd,
        node_teleopt,
        april_node
    ])