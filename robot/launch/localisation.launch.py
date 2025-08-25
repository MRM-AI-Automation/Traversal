from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static Transform Publisher for IMU
     
        
        # Navsat Transform Node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'frequency': 30.0},
                {'queue_size': 30},
                {'magnetic_declination_radians': 0.0},
                {'yaw_offset': 0.0},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'zed_camera_link'},
                {'world_frame': 'map'},
                {'publish_filtered_gps': True}
            ],
            remappings=[
                ('/imu', '/imu/data_raw'),
                ('/gps/fix', '/fix'),
                ('/odometry/gps', '/odom_bad')
            ],
            respawn=True,
        ),

        # RTAB-Map Node
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
                {'subscribe_imu': True}
            ],
            remappings=[
                ('rgb/image', '/zed/zed_node/left/image_rect_color'),
                ('rgb/camera_info', '/zed/zed_node/left/camera_info'),
                ('depth/image', '/zed/zed_node/depth/depth_registered'),
                ('/imu', '/imu/data_raw'),
                ('/odom', '/odometry/filtered'),
            ],
            arguments=['-d'],
        ),

        # Odom Frame ID Changer Node
        Node(
            package='gps',
            executable='odomframeidchanger',
            name='odomframeidchanger',
            output='screen'
        ),

        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'frequency': 30.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'publish_tf': False,
                'odom_frame': 'odom',
                'base_link_frame': 'zed_camera_link',
                'world_frame': 'odom',
                'odom_topic': 'odom_combined',
                'imu0': '/imu/data_raw',
                'imu0_config': [
                    False, False, False,
                    True, True, True,
                    True, True, True,
                    True, True, True,
                    True, True, True
                ],
                'imu0_differential': True,
                'imu0_queue_size': 10,
                'odom0': '/temp',
                'odom0_config': [
                    True, True, False,
                    False, False, False,
                    False, False, False,
                    False, False, False,
                    False, False, False
                ],
                'odom0_queue_size': 10,
                'odom0_differential': True,
            }],
        )
    ])
