"""
Copy from `/opt/ros/galactic/share/slam_toolbox/launch/online_sync_launch.py`
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    namespace = LaunchConfiguration('namespace', default="")
    map_name = LaunchConfiguration('map_name', default="map")
    node_name = LaunchConfiguration('node_name', default="slam_toolbox")

    declare_use_sim_time_argument = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_slam_params_file_cmd = DeclareLaunchArgument('slam_params_file')

    start_sync_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        namespace=namespace,
        name=node_name,
        output='screen',
        remappings=[
            ("map", map_name)
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
