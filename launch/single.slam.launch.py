"""
world_name: cloister, cloister_asphalt, gallery, playpen, playpen_asphalt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    """
    # -- vars
    folder_bringup = get_package_share_directory('my_ros2_bringup')
    folder_robot = get_package_share_directory('my_ros2_robot_gazebo')

    # -- IncludeLaunchDescription
    # -- gazebo
    launch_world = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_robot, 'launch', '_gazebo.launch.py')),
        launch_arguments={'name': "cloister"}.items(),
    )

    # -- spawn robot
    launch_spawn = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_robot, 'launch', '_spawn_by_xacro.launch.py')),
        launch_arguments={
            "use_sim_time": "true",
            "name": "r1",
            "ns": "",
            "x": "0.0",
            "y": "0.0",
            "z": "0.2",
            "qx": "0.0",
            "qy": "0.0",
            "qz": "0.0",
            "qw": "1.0",
        }.items()
    )

    # -- slam
    path_params = os.path.join(folder_bringup, 'config', 'mapper_params_online_sync.yaml')
    launch_slam = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_bringup, 'launch', '_online_sync_launch.py')),
        launch_arguments={
            "slam_params_file": path_params,
            "namespace": "",
            "map_name": "map1"
        }.items()
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_world)
    ld.add_action(launch_spawn)
    ld.add_action(launch_slam)

    return ld
