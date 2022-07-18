"""
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import numpy as np
from quaternions import Quaternion

FOLDER_BRINGUP = get_package_share_directory('my_ros2_bringup')
FOLDER_ROBOT = get_package_share_directory('my_ros2_robot_gazebo')
FOLDER_MAP_MERGE = get_package_share_directory('multirobot_map_merge')
USE_SIM_TIME = LaunchConfiguration('use_sim_time', default='true')

def get_launch_robots(settings: list) -> list:
    output = list()
    for s in settings:
        q = Quaternion.from_euler([0, 0, np.deg2rad(s['deg'])])
        d = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_ROBOT, 'launch', '_spawn_by_xacro.launch.py')),
            launch_arguments={
                "use_sim_time": USE_SIM_TIME,
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
    node_tf1 = Node(
        package='tf2_ros',
        namespace='static_tf',
        executable='static_transform_publisher',
        name='pub1',
        arguments=["10", "-20", "0", "0", "0", "0", "world", "robot1/map"],
    )

    node_tf2 = Node(
        package='tf2_ros',
        namespace='static_tf',
        executable='static_transform_publisher',
        name='pub2',
        arguments=["-10", "-20", "0", "0", "0", "0", "world", "robot2/map"],
    )

    # -- slam
    launch_slams = get_launch_slams(settings=settings)

    # -- rviz
    launch_rviz = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_BRINGUP, 'launch', '_rviz_multi.launch.py')),
    )

    # -- map merge
    node_map_merged = Node(
        package="multirobot_map_merge",
        name="map_merge",
        namespace="/",
        executable="map_merge",
        parameters=[
            os.path.join(FOLDER_BRINGUP, "config", "map_merged_w_init.yaml"),
            # os.path.join(FOLDER_BRINGUP, "config", "map_merged_wo_init.yaml"),
        ],
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_world)
    ld.add_action(node_tf1)
    ld.add_action(node_tf2)
    for i in launch_robots:
        ld.add_action(i)
    for i in launch_slams:
        ld.add_action(i)
    ld.add_action(launch_rviz)
    ld.add_action(node_map_merged)

    return ld
