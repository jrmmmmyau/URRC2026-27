import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    
    config=os.path.join(
        get_package_share_directory('apriltag_localization'),'config','apriltag.yaml'
    )

    return LaunchDescription([
        Node(
            package='image_proc',
            executable='rectify_node',
            remappings=[('image', '/camera/front_camera/color/image_raw'),
                        ('camera_info', '/camera/front_camera/color/camera_info'),]
        ),
        
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            parameters=[config],
            remappings=[('image_rect', '/image_rect'),
                        ('camera_info', '/camera/front_camera/color/camera_info'),]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_mount_tf',
        #     arguments= ['--x', '0.3',
        #                 '--y', '0.0', 
        #                 '--z', '0.5',
        #                 '--yaw', '0.0', 
        #                 '--pitch', '0.0', 
        #                 '--roll', '0.0',
        #                 '--frame-id',
        #                 'base_link',
        #                 '--child-frame-id', 
        #                 'camera_link']),
        Node(
            package='apriltag_localization',
            executable='tag_localizer',
            parameters=[{
                'map_frame': 'map',
                'base_frame': 'base_link',
                'tag_frame_prefix': 'tag_',
                'position_stddev': 0.05,
                'angle_stddev': 0.05,

                # Where each tag is on the field (change)
                'tag_ids':   [0],
                'tag_x':     [0.0],
                'tag_y':     [0.0],
                'tag_z':     [0.0],
                'tag_roll':  [0.0],
                'tag_pitch': [0.0],
                'tag_yaw':   [0.0],
            }],
        ),
    ])