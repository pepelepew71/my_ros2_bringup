"""
world_name: cloister, cloister_asphalt, gallery, playpen, playpen_asphalt
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
    folder_robot = get_package_share_directory('my_ros2_robot_gazebo')

    # -- IncludeLaunchDescription
    launch_world = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_robot, 'launch', '_gazebo.launch.py')),
        launch_arguments={'name': "cloister"}.items(),
    )

    launch_spawn = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_robot, 'launch', '_spawn_by_xacro.launch.py')),
        launch_arguments={
            "use_sim_time": "true",
            "name": "mrobot",
            "x": "0.0",
            "y": "0.0"
        }.items(),
    )

    launch_slam = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_bringup, 'launch', '_slam.launch.py')),
    )

    launch_nav = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_bringup, 'launch', '_nav.launch.py')),
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_world)
    ld.add_action(launch_spawn)
    ld.add_action(launch_slam)
    ld.add_action(launch_nav)

    return ld
