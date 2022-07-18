"""
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

FOLDER_BRINGUP = get_package_share_directory('my_ros2_bringup')

def generate_launch_description():
    """
    """
    node_rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(FOLDER_BRINGUP, 'rviz', 'nav.rviz')]
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(node_rviz2)

    return ld
