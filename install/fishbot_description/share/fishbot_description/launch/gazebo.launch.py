from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro


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
        'fishbot.urdf.xacro'
    )

    # Process xacro
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s',
            'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ],
        output='screen'
    )

    # Spawn Robot
    spawn_fishbot = Node(
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

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    tail_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'tail_position_controller'
        ],
        output='screen'
    )

    left_fin_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['left_fin_controller'],
    output='screen'
    )

    right_fin_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['right_fin_controller'],
    output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_fishbot,
        joint_state_broadcaster_spawner,
        tail_controller_spawner,
        left_fin_controller_spawner,
        right_fin_controller_spawner,
    ])