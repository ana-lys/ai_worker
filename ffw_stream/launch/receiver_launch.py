import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ffw_stream')
    
    left_config = os.path.join(pkg_dir, 'config', 'camera_left_config.yaml')
    right_config = os.path.join(pkg_dir, 'config', 'camera_right_config.yaml')

    return LaunchDescription([
        Node(
            package='ffw_stream',
            executable='realsense_udp_receiver',
            name='left_receiver',
            namespace='camera_left',
            parameters=[left_config, {'camera_name': 'Left Camera'}],
            output='screen'
        ),
        Node(
            package='ffw_stream',
            executable='realsense_udp_receiver',
            name='right_receiver',
            namespace='camera_right',
            parameters=[right_config, {'camera_name': 'Right Camera'}],
            output='screen'
        )
    ])
