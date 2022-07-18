"""
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

FOLDER_BRINGUP = get_package_share_directory('my_ros2_bringup')

def generate_launch_description():
    """
    """
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    node_slam = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        output='screen',
        namespace="",
        parameters=[
            {'use_sim_time': use_sim_time},
            {'base_frame': 'base_link'},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
        ],
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(node_slam)

    return ld
