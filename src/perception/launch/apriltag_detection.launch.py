import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'apriltag.yaml'
    )

    return LaunchDescription([
        Node(
            package='image_proc',
            executable='rectify_node',
            remappings=[
                ('image', '/camera/top_camera/color/image_raw'),
                ('camera_info', '/camera/top_camera/color/camera_info'),
            ],
            arguments=['--ros-args', '--log-level', 'error'],
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[config],
            remappings=[
                ('image_rect', '/image_rect'),
                ('camera_info', '/camera/top_camera/color/camera_info'),
            ],
            arguments=['--ros-args', '--log-level', 'error'],
        ),
    ])
