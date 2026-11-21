import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('apriltag_localization'),
        'config',
        'localization.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('observations_topic', default_value='/perception/tag/observations'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='apriltag_localization',
            executable='tag_localizer',
            name='tag_localizer',
            parameters=[config, {
                'base_frame': ParameterValue(LaunchConfiguration('base_frame'), value_type=str),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[('tag/observations', LaunchConfiguration('observations_topic'))],
        ),
    ])
