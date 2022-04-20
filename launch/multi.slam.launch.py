"""
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import numpy as np
from quaternions import Quaternion

FOLDER_BRINGUP = get_package_share_directory('my_ros2_bringup')
FOLDER_ROBOT = get_package_share_directory('my_ros2_robot_gazebo')
FOLDER_MAP_MERGE = get_package_share_directory('multirobot_map_merge')

def get_launch_robots(settings: list) -> list:
    output = list()
    for s in settings:
        q = Quaternion.from_euler([0, 0, np.deg2rad(s['deg'])])
        d = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_ROBOT, 'launch', '_spawn_by_xacro.launch.py')),
            launch_arguments={
                "use_sim_time": "true",
                "name": s['name'],
                "ns": s['name'],
                "x": str(s['x']),
                "y": str(s['y']),
                "z": "0.2",
                "qx": str(q.x),
                "qy": str(q.y),
                "qz": str(q.z),
                "qw": str(q.w),
            }.items()
        )
        output.append(d)
    return output

def get_launch_slams(settings: list) -> list:
    output = list()
    for s in settings:
        path = os.path.join(FOLDER_BRINGUP, 'config', f'mapper_params_online_sync.{s["name"]}.yaml')
        d = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_BRINGUP, 'launch', '_online_sync_launch.py')),
            launch_arguments={
                "slam_params_file": path,
                "namespace": s["name"],
            }.items()
        )
        output.append(d)
    return output

def generate_launch_description():
    """
    """

    # -- IncludeLaunchDescription
    # -- gazebo, world_name: cloister, cloister_asphalt, gallery, playpen, playpen_asphalt
    launch_world = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_ROBOT, 'launch', '_gazebo.launch.py')),
        launch_arguments={'name': "cloister"}.items(),
    )

    # -- spawn robot
    settings = [
        {'name': "robot1", "x":  1.0, "y": 0.0, "deg": 0.0},
        {'name': "robot2", "x": -1.0, "y": 0.0, "deg": 180.0},
    ]
    launch_robots = get_launch_robots(settings=settings)

    # -- static tf publisher
    node_tf_pub = Node(
        package='tf2_ros',
        namespace='',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )

    # -- slam
    launch_slams = get_launch_slams(settings=settings)

    # -- map merge
    launch_map_merge = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_MAP_MERGE, 'launch', 'map_merge.launch.py')),
        launch_arguments={'known_init_poses': "true"}.items(),
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_world)
    ld.add_action(node_tf_pub)
    for i in launch_robots:
        ld.add_action(i)
    for i in launch_slams:
        ld.add_action(i)
    ld.add_action(launch_map_merge)

    return ld
