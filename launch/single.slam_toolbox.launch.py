"""
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

FOLDER_BRINGUP = get_package_share_directory('my_ros2_bringup')
FOLDER_ROBOT = get_package_share_directory('my_ros2_robot_gazebo')
USE_SIM_TIME = LaunchConfiguration('use_sim_time', default='true')

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
    launch_spawn = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_ROBOT, 'launch', '_spawn_by_xacro.launch.py')),
        launch_arguments={
            "use_sim_time": USE_SIM_TIME,
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
    launch_slam = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_BRINGUP, 'launch', '_slam_toolbox.launch.py')),
    )

    # -- rviz
    launch_rviz = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(FOLDER_BRINGUP, 'launch', '_rviz_slam.launch.py')),
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_world)
    ld.add_action(launch_spawn)
    ld.add_action(launch_slam)
    ld.add_action(launch_rviz)

    return ld
