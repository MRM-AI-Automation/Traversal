from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'magnetic_declination_radians': 0.0},
                {'yaw_offset': 0.0},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'zed_imu_link'},
                {'world_frame': 'map'},
                {'publish_filtered_gps': True}
            ],
            remappings=[
                ('/imu', '/zed/zed_node/imu/data'),
                ('/gps/fix', '/fix'),
                ('/odometry/gps', '/odom')
            ],
            respawn=True,
        )
    ])
