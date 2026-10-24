from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('start'),
        'launch',
        'sim.launch.py'
    )

    navigation_launch = os.path.join(
        get_package_share_directory('navigation_server'),
        'launch',
        'nav_test.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch)
        ),
    ])
