from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        ),
    ])