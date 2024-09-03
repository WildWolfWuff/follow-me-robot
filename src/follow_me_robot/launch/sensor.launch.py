import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,InvalidLaunchFileError
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    log_level=LaunchConfiguration('log_level')
    use_sim_time=LaunchConfiguration('use_sim_time')
    config_path=LaunchConfiguration('config_path',default=os.path.join(pkg_path,'config','sensor_config.yaml'))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='log level'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use sim time'),
        DeclareLaunchArgument(
            'config_path',
            default_value=config_path,
            description='path to sensor config file'),
        # Launch the follow_me_teleop node for controlling the robot with joystick
        # ros2 run follow_me_teleop follow_me_teleop --ros-args --params-file `ros2 pkg prefix follow_me_robot`/share/follow_me_robot/config/sensor_config.yaml
        Node(
            package='follow_me_teleop',
            executable='follow_me_teleop',
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[config_path],
        ),
        # Launch the apriltag node for detecting the apriltags with camera
        # ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/cam/front/image_raw -r camera_info:=/cam/front/camera_info -r /tf:=/tag/tf -r /detections:=/tag/detections --params-file `ros2 pkg prefix follow_me_robot`/share/follow_me_robot/config/sensor_config.yaml
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            remappings=[
                ('image_rect','/cam/front/image_raw'),
                ('camera_info','/cam/front/camera_info'),
                ('/detections','/tag/detections'),
            ],
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[config_path]
        ),
        # Launch the follow_me_path_builder node for creating a gola_pose from the apriltag detection
        # ros2 run follow_me_path_builder follow_me_path_builder --ros-args -r /tf:=/tag/tf --params-file `ros2 pkg prefix follow_me_robot`/share/follow_me_robot/config/sensor_config.yaml
        Node(
            package='follow_me_path_builder',
            executable='follow_me_path_builder',
            parameters=[config_path],
            arguments=["--ros-args", '--log-level', log_level],
        ),
        # TODO: add the other sensor nodes here
    ])