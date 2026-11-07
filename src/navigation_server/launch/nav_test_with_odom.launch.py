from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('navigation_server')
    rviz_config = os.path.expanduser('~/URRC2026-27/rviz/nav_odom.rviz')
    pure_pursuit_arg = DeclareLaunchArgument('pure_pursuit', default_value='true')

    return LaunchDescription([
        pure_pursuit_arg,
        # A* navigation server
        Node(
            package='navigation_server',
            executable='navsrv',
            name='navigation_server',
            output='screen',
        ),

        # Monitors the map, goal, and current path
        Node(
            package='navigation_server',
            executable='map_verifier_server',
            name='map_verifier_server',
            output='screen',
        ),

        # Converts the current path into velocity commands
        Node(
            package='navigation_server',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher',
            output='screen',
            parameters=[
                {'pure_pursuit': ParameterValue(LaunchConfiguration('pure_pursuit'), value_type=bool)}
                
            ],
        ),

        # Connects map to odom for RViz visualization
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
            output='screen',
        ),

        # Opens RViz with the testing configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
