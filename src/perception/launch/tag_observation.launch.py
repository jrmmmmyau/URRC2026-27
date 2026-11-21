import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'tag_observer.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('detections_topic', default_value='/detections'),
        DeclareLaunchArgument('observations_topic', default_value='/perception/tag/observations'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='perception', executable='tag_observer', name='tag_observer',
            output='screen',
            parameters=[config, {
                'output_frame': ParameterValue(LaunchConfiguration('base_frame'), value_type=str),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[
                ('detections', LaunchConfiguration('detections_topic')),
                ('tag/observations', LaunchConfiguration('observations_topic')),
            ],
        ),
    ])
