import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node, PushRosNamespace
import xacro

def generate_launch_description():
    # define package name
    pkg_name = 'followme_robot_model'
    pkg_path = get_package_share_directory(pkg_name)
    
    # define and set path to xacro model file
    model_file_path = 'robot/robot.urdf.xacro'
    xacro_file = os.path.join(pkg_path,model_file_path)
    
    # compile model
    bot_description_compiled = xacro.process_file(xacro_file).toxml()
    
    # define start script for robot state publisher to publish the compiled robto description
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': bot_description_compiled,
        'use_sim_time': True}] # add other parameters here if required
    )
    
    # define start script for gazebo ui
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    # define start script to spawn the robot from the robot description and name it mecanum-bot
    node_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'mecanum-bot'],
                    output='screen')
    # TODO add rviz config file
    # see example: https://github.com/turtlebot/turtlebot4_desktop/blob/humble/turtlebot4_viz/rviz/robot.rviz
    # rviz2_config = PathJoinSubstitution(
    #     [pkg_path, 'rviz', 'model.rviz'])
    
    # namespace = LaunchConfiguration('namespace')
    
    # node_rviz = GroupAction([
    #     PushRosNamespace(namespace),

    #     Node(package='rviz2',
    #          executable='rviz2',
    #          name='rviz2',
    #          arguments=['-d', 'bot.rviz'],
    #          parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #          remappings=[
    #             ('/tf', 'tf'),
    #             ('/tf_static', 'tf_static')
    #          ],
    #          output='screen'),

    #     # Delay launch of robot description to allow Rviz2 to load first.
    #     # Prevents visual bugs in the model.
    #     TimerAction(
    #         period=3.0,
    #         actions=[
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource([description_launch]),
    #                 launch_arguments=[('model', LaunchConfiguration('model'))],
    #                 condition=IfCondition(LaunchConfiguration('description'))
    #             )])
    # ])
    
    # retuns the defined launch scripts
    return  LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        node_spawn_entity,
        # node_rviz2
    ])