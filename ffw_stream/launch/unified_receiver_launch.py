import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ffw_stream')

    left_config = os.path.join(pkg_dir, 'config', 'camera_left_config.yaml')
    right_config = os.path.join(pkg_dir, 'config', 'camera_right_config.yaml')

    return LaunchDescription([
        # We launch a single unified node using the left_receiver name.
        # This receiver will now spawn RealSense AND ZED threads internally and output the single unified dashboard.
        Node(
            package='ffw_stream',
            executable='realsense_udp_receiver',
            name='unified_dashboard_receiver',
            namespace='cameras',
            parameters=[left_config, {'camera_name': 'Unified Dashboard'}],
            output='screen'
        )
    ])
