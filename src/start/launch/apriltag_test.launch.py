import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    start_share = get_package_share_directory('start')
    camera_share = get_package_share_directory('realsense2_camera')
    perception_share = get_package_share_directory('perception')
    apriltag_share = get_package_share_directory('apriltag_localization')

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('observations_topic', default_value='/perception/tag/observations'),
        DeclareLaunchArgument(
            'color_profile',
            default_value='848x480x30',
            description='RealSense RGB stream resolution and frame rate',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(start_share, 'launch', 'sim.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(camera_share, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'camera_namespace': 'camera',
                'camera_name': 'top_camera',
                'enable_color': 'true',
                'enable_depth': 'false',
                'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    perception_share, 'launch', 'apriltag_detection.launch.py'
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_share, 'launch', 'tag_observation.launch.py')
            ),
            launch_arguments={
                'base_frame': LaunchConfiguration('base_frame'),
                'observations_topic': LaunchConfiguration('observations_topic'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(apriltag_share, 'launch', 'apriltag.launch.py')
            ),
            launch_arguments={
                'base_frame': LaunchConfiguration('base_frame'),
                'observations_topic': LaunchConfiguration('observations_topic'),
            }.items(),
        ),
    ])
