import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    
    config=os.path.join(
        get_package_share_directory('apriltag_localization'),'config','apriltag.yaml'
    )

    # tag_map = os.path.join(
    #     get_package_share_directory('apriltag_localization'),
    #     'config', 'tag_map.yaml')

    return LaunchDescription([
        Node(
            package='image_proc',
            executable='rectify_node',
            remappings=[('image', '/camera/camera/color/image_raw'),
                        ('camera_info', '/camera/camera/color/camera_info'),]
        ),
        
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            parameters=[config],
            remappings=[('image_rect', '/image_rect'),
                        ('camera_info', '/camera/camera/color/camera_info'),]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_mount_tf',
            arguments= ['--x', '0.3',
                        '--y', '0.0', 
                        '--z', '0.5',
                        '--yaw', '0.0', 
                        '--pitch', '0.0', 
                        '--roll', '0.0',
                        '--frame-id',
                        'base_link',
                        '--child-frame-id', 
                        'camera_link']),
        # Node(
        #     package='apriltag_localization',
        #     executable='tag_localizer',
        #     parameters=[tag_map],
        # ),
    ])