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
    tel_config_path=LaunchConfiguration('tel_config_path',default=os.path.join(pkg_path,'config','teleop_config.yaml'))
    # print(os.path.join(pkg_path,'config','apritag.yaml'))
    tag_config_path=LaunchConfiguration('tag_config_path',default=os.path.join(pkg_path,'config','apritag.yaml'))
    declare_log_level_cmd=DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')
    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd=DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use sim time')
    return LaunchDescription([
        declare_log_level_cmd,
        declare_use_sim_time_cmd,
        DeclareLaunchArgument('tel_config_path',default_value=tel_config_path,description='path to teleop config file'),
        DeclareLaunchArgument('tag_config_path',default_value=tag_config_path,description='path to apriltag config file'),
        # ros2 run follow_me_teleop follow_me_teleop --ros-args --params-file `ros2 pkg prefix follow_me_teleop`/share/follow_me_teleop/config/teleop_config.yaml
        Node(
            package='follow_me_teleop',
            executable='follow_me_teleop',
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[os.path.join(pkg_path, 'config', 'teleop_config.yaml')],
        ),
        TimerAction(period=10.0,
                    cancel_on_shutdown=True,
                    actions=[
            # ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/cam/front/image_raw -r camera_info:=/cam/front/camera_info -r /tf:=/tag/tf -r /detections:=/tag/detections --params-file `ros2 pkg prefix follow_me_robot`/share/follow_me_robot/config/apritag.yaml
            Node(
            package='apriltag_ros',
            executable='apriltag_node',
            remappings=[
                ('image_rect','/cam/front/image_raw'),
                ('camera_info','/cam/front/camera_info'),
                ('/tf','/tag/tf'),
                ('/detections','/tag/detections'),
            ],
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[tag_config_path]
            )
        ])

    ])