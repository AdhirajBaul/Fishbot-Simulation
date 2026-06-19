from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_path = get_package_share_directory('fishbot_description')

    world_file = os.path.join(
        pkg_path,
        'worlds',
        'ocean.world'
    )

    urdf_file = os.path.join(
        pkg_path,
        'urdf',
        'fishbot.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([

        # Start Gazebo
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_file,
                '-s',
                'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Publish robot TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description}
            ],
            output='screen'
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity',
                'fishbot',
                '-topic',
                'robot_description'
            ],
            output='screen'
        )

    ])
