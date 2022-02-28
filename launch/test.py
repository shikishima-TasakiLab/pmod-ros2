import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('pmod_ros'), 'config')

    return LaunchDescription([
        Node(
            namespace='pmod',
            package='pmod_ros',
            executable='pmod',
            parameters=[
                {'hz': 5.0},
                {'sub_queue_size': 10},
                {'pub_queue_size': 2},
                {'use_optical_frame': False},
                {'pmod_config_path': os.path.join(config_dir, 'pmod-5class.yaml')},
                {'use_sync': False},
                {'pub_seg_id': True},
                {'pub_seg_color': False},
                {'pub_depth': True},
                {'pub_points_ground': False},
                {'pub_points_no_ground': False},
                {'pub_points_static': False},
                {'pub_points_dynamic': True},
            ],
        ),
        Node(
            namespace='pmod',
            package='pmod_ros',
            executable='dynamic2noground',
            parameters=[
                {'init_maps': [
                    "/workspace/src/pmod_ros/maps/noground.pcd",
                ]},
                {'init_frame_id': 'map'},
                {'radius': 5.0},
            ],
            remappings=[
                ('points_no_ground', '/points_no_ground')
            ]
        ),
        Node(
            namespace='pmod',
            package='pmod_ros',
            executable='imgpub',
            parameters=[
                {'camera_path': '/workspace/src/pmod_ros/test/test_camera.png'},
                {'sparse_depth_path': '/workspace/src/pmod_ros/test/test_sparse_depth.exr'},
                {'Fx': 788.629315},
                {'Cx': 687.158398},
                {'Fy': 786.382230},
                {'Cy': 317.752196},
                {'frame_id': 'camera'},
            ],
        )
    ])
