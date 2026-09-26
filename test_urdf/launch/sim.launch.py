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
    # convert to gz format
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_pos/fl_steering@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_pos/fr_steering@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_pos/rl_steering@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_pos/rr_steering@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_vel/fl_wheel@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_vel/fr_wheel@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_vel/rl_wheel@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_vel/rr_wheel@std_msgs/msg/Float64]gz.msgs.Double',
            '/world/empty/model/lunabot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/lunabot/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # run driver code
    driver_node = Node(
        package='executive',
        executable='driver_node',
        output='screen'
    )

    return LaunchDescription([rsp_node, gazebo, spawn_node, bridge_node, driver_node])
