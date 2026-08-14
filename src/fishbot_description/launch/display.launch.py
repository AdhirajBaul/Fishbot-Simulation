from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('fishbot_description')

    urdf_file = os.path.join(
        pkg_path,
        'urdf',
        'fishbot.urdf.xacro'
    )

    rviz_config = os.path.join(
        pkg_path,
        'rviz',
        'fishbot.rviz'
    )

    # Process Xacro
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    # Publishes TF from the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ],
        output='screen'
    )

    # GUI sliders for moving revolute joints
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])