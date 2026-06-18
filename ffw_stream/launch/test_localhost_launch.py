import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ffw_stream')
    config_file = os.path.join(pkg_dir, 'config', 'test_localhost_config.yaml')

    return LaunchDescription([
        Node(
            package='ffw_stream',
            executable='realsense_udp_streamer',
            name='localhost_streamer',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='ffw_stream',
            executable='realsense_udp_receiver',
            name='localhost_receiver',
            namespace='camera_localhost',
            parameters=[config_file, {'camera_name': 'Localhost Camera'}],
            output='screen'
        )
    ])
