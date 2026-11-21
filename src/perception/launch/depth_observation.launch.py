import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'depth.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_image_topic', default_value='/camera/camera/depth/image_rect_raw',
            description='Depth image topic, encoded as 16UC1 or 32FC1',
        ),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/depth/camera_info',
            description='Calibration topic matching the depth image',
        ),
        DeclareLaunchArgument('points_topic', default_value='/perception/depth/points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='perception', executable='depth_observer', name='depth_observer',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[
                ('depth/image', LaunchConfiguration('depth_image_topic')),
                ('depth/camera_info', LaunchConfiguration('camera_info_topic')),
                ('depth/points', LaunchConfiguration('points_topic')),
            ],
        ),
    ])
