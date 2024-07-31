import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,InvalidLaunchFileError
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

# launch the robot state publisher
def generate_launch_description():
    # define package name
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    # namespace = 'Follow-Me-Robot'
    
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
        parameters=[
            {
                'robot_description': bot_description_compiled,
                'use_sim_time': True
             }] # add other parameters here if required
        )
    
    return LaunchDescription([
        node_robot_state_publisher,
    ])
