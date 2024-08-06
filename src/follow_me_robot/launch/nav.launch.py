import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,InvalidLaunchFileError
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,ThisLaunchFileDir

def generate_launch_description():
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    use_sim_time=LaunchConfiguration('use_sim_time',default='false')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                  default=os.path.join(pkg_path, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='follow_me_mapper_lds_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # ros2 run follow_me_teleop follow_me_teleop --ros-args --params-file `ros2 pkg prefix follow_me_teleop`/share/follow_me_teleop/config/teleop_config.yaml
    return LaunchDescription([
        DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='use sim time'),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '--log-level warn']),
                DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, 
                       '-publish_period_sec', publish_period_sec,
                       '--log-level warn'])
    ])