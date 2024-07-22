import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # define package name
    pkg_name = 'follow_me_robot'
    pkg_path = get_package_share_directory(pkg_name)
    
    # define start script for robot state publisher to publish the compiled robto description
    node_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_path, 'launch', 'robot.launch.py'
        )))
    # define start script for gazebo ui

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
        )

    # define start script to spawn the robot from the robot description and name it mecanum-bot
    node_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'mecanum-bot'],
                    output='screen')
    # ros2 run apriltag_ros apriltag_node --ros_args -remap image_rect:=/rrbot/cam/front -remap camera_info:=/rrbot/cam/front/camera_info --params-file /home/dev/follow-me-robot/install/follow_me_robot/share/follow_me_robot/config/apriltag.yaml
    # ros2 run apriltag_ros apriltag_node --ros-args -remap image_rect:=/rrbot/cam/front/image_raw -remap camera_info:=/rrbot/cam/front/camera_info --params-file `ros2 pkg prefix follow_me_robot`/share/follow_me_robot/config/apritag.yaml
    april_node=Node(
      package='apriltag_ros',
      executable='apriltag_node',
      # namespace='followme',
      # name="apriltag_detection",
      remappings=[
        ('image_rect','/rrbot/cam/front/image_raw'),
        ('camera_info','/rrbot/cam/front/camera_info'),
        ('/tf','/tf/tag')
      ],
      parameters=[
        os.path.join(pkg_path,'config','apritag.yaml')
      ]
    )

    # retuns the defined launch scripts
    return  LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        node_spawn_entity,
        april_node,
    ])