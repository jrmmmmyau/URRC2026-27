from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_server',
            executable='navsrv',
            name='navigation_server',
            output='screen',
        ),
        Node(
            package='navigation_server',
            executable='map_verifier_server',
            name='map_verifier_server',
            output='screen',
        ),
        Node(
            package='navigation_server',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher',
            output='screen',
        ),
    ])