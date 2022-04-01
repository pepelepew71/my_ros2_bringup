"""
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    """
    # -- vars
    folder_bringup = get_package_share_directory('my_ros2_bringup')
    folder_slam_toolbox = get_package_share_directory('slam_toolbox')

    # -- IncludeLaunchDescription
    path_params = os.path.join(folder_bringup, 'config', 'mapper_params_online_sync.yaml')
    launch_slam = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_slam_toolbox, 'launch', 'online_sync_launch.py')),
        launch_arguments={"slam_params_file": path_params}.items()
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_slam)

    return ld
