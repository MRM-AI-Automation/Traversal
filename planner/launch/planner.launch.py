from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # YOLO inference node

    zed2_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("zed_wrapper"),
                        'launch/zed_camera.launch.py')
        ),
        launch_arguments=[('camera_model', 'zed2i')]
    )


    # IMU conversion node
    imu_conversion_node = Node(
        package='robot',
        executable='imu_conversion_node',
        output='log'
    )
    
    # Return the LaunchDescription with all included nodes
    return LaunchDescription([
        zed2_launch_file,
        imu_conversion_node
    ])