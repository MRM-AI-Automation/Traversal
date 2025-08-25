from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[
                {'frame_id': 'zed_camera_link'},
                {'approx_sync': True}  # For synchronized RGB-D odometry
            ],
            remappings=[
                ('rgb/image', '/zed/zed_node/left/image_rect_color'),
                ('rgb/camera_info', '/zed/zed_node/left/camera_info'),
                ('depth/image', '/zed/zed_node/depth/depth_registered')
            ],
            arguments=['-d'],
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                {'queue_size': 30},
                {'frame_id': 'zed_camera_link'},
                {'subscribe_depth': True},
                {'subscribe_rgbd': False},
                {'approx_sync': True},
                {'Rtabmap/PublishOccupancyGrid': True},
                {'subscribe_odom_info': True}  # To receive odometry info from rgbd_odometry
            ],
            remappings=[
                ('odom', '/odom'),  # Output from rgbd_odometry
                ('rgb/image', '/zed/zed_node/left/image_rect_color'),
                ('rgb/camera_info', '/zed/zed_node/left/camera_info'),
                ('depth/image', '/zed/zed_node/depth/depth_registered'),
                ('/imu', '/imu/data_raw'),
            ],
            arguments=['-d'],
        )
    ])
