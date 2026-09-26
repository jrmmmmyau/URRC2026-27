import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share=get_package_share_directory('test_urdf')
    urdf_path=os.path.join(pkg_share, 'urdf', 'basic_robot.urdf')
    ros_gz_sim=get_package_share_directory('ros_gz_sim')
    rsp_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['cat ',urdf_path])}]
    )

    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ros_gz_sim, 'launch','gz_sim.launch.py')]),
        launch_arguments={'gz_args':'empty.sdf -r'}.items()
    )
    spawn_node=Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic','robot_description','-name','lunabot','-z','0.5'],
        output='screen'
    )
    return LaunchDescription([rsp_node, gazebo, spawn_node])