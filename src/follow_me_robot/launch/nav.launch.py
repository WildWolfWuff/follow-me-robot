import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,InvalidLaunchFileError
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription, TimerAction,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    # ros2 run follow_me_teleop follow_me_teleop --ros-args --params-file `ros2 pkg prefix follow_me_teleop`/share/follow_me_teleop/config/teleop_config.yaml
    log_level = 'info'
    remappings = [('/tf', 'tf'),
              ('/tf_static', 'tf_static')]
    controler= Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
    # nav2_bringup=IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(os.path.join(
    #     get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py')
    #   ),launch_arguments={
    #     'use_sim_time':'True',
    #     # 'slam':'False',
    #     'map':os.path.join(pkg_path,'maps','map.yaml'),
    #     'params_file': os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
    #     'log_level': 'info'
    #   }.items()
    # )
    # slma_toolbox=IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(os.path.join(
    #     get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')
    #   ),launch_arguments={
    #     'use_sim_time':'True'
    #   }.items()
    # )
    return LaunchDescription([
        nav2_bringup,
        slma_toolbox
    ])