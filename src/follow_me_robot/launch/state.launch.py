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
    declare_log_level_cmd=DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')
    # namespace = 'Follow-Me-Robot'
    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd=DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use sim time')
    # define start script for robot state publisher to publish the compiled robto description
    model_file_path = 'urdf/robot.urdf.xacro'
    xacro_file = os.path.join(pkg_path,model_file_path)

    # compile model
    # cd `ros2 pkg prefix follow_me_teleop`/urdf; xacro robot.urdf.xacro
    bot_description_compiled = xacro.process_file(xacro_file).toxml()
    if bot_description_compiled is None:
        raise InvalidLaunchFileError("Error while compile robot description")
    # define start script for robot state publisher to publish the compiled robto description
    
    node_robot_state_publisher = Node(
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
    
    return LaunchDescription([
        declare_log_level_cmd,
        declare_use_sim_time_cmd,
        node_robot_state_publisher,
    ])