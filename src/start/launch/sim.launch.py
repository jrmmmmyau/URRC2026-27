"""Backward-compatible entry point for the flat arena and sensor-equipped Melbot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_path = os.path.join(
        get_package_share_directory('start'), 'launch', 'teleop.launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)),
    ])
