"""
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

FOLDER_BRINGUP = get_package_share_directory('my_ros2_bringup')

def generate_launch_description():
    """
    """
    path_params = os.path.join(FOLDER_BRINGUP, 'config', 'mapper_params_online_sync.yaml')
    launch_slam = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_BRINGUP, 'launch', '_online_sync_launch.py')),
        launch_arguments={
            "slam_params_file": path_params,
        }.items()
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_slam)

    return ld
