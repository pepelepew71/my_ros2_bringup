"""
world_name: cloister, cloister_asphalt, gallery, playpen, playpen_asphalt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    """
    # -- vars
    folder_bringup = get_package_share_directory('my_ros2_bringup')

    # -- Node
    node_rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(folder_bringup, 'rviz', 'nav.rviz')]
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(node_rviz2)

    return ld
