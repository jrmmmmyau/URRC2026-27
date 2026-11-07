import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    start_share = get_package_share_directory('start')
    nav_share = get_package_share_directory('navigation_server')
    PURE_PURSUIT = 'false' 

    pure_pursuit_arg = DeclareLaunchArgument(
        'pure_pursuit', default_value='true',
        description='true = pure pursuit controller, false = old controller'
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(start_share, 'launch', 'sim.launch.py')
        )
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'nav_test_with_odom.launch.py')
        ),
        launch_arguments={'pure_pursuit': PURE_PURSUIT}.items()
    )

    return LaunchDescription([pure_pursuit_arg, sim, nav])