from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Unified receiver: auto-opens the known ports and displays whatever streams are present.
        Node(
            package='ffw_stream',
            executable='realsense_udp_receiver',
            name='unified_dashboard_receiver',
            namespace='cameras',
            parameters=[{'headless': False}],
            output='screen'
        )
    ])
