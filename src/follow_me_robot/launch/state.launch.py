import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,InvalidLaunchFileError
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

# launch the robot state publisher
def generate_launch_description():
    # define package name
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    log_level=LaunchConfiguration('log_level')
    use_sim_time=LaunchConfiguration('use_sim_time')
    
    model_file_path = 'urdf/robot.urdf.xacro'
    xacro_file = os.path.join(pkg_path,model_file_path)
    # compile xacro model to urdf
    # cd `ros2 pkg prefix follow_me_teleop`/urdf; xacro robot.urdf.xacro
    bot_description_compiled = xacro.process_file(xacro_file).toxml()
    if bot_description_compiled is None:
        raise InvalidLaunchFileError("Error while compile robot description")

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='log level'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use sim time'),
        # robot state publisher node, for publishing the compiled robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                {
                    'robot_description': bot_description_compiled,
                    'use_sim_time': use_sim_time
                 }] # add other parameters here if required
            )
    ])
