import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ffw_stream',
            executable='realsense_udp_streamer',
            name='test_rs2_streamer',
            parameters=[{
                # Leave device_id empty to grab ANY available RealSense camera
                'device_id': '',
                
                # Try just a single low-bandwidth stream to eliminate USB bottlenecks
                'enable_rgb': False,
                'enable_depth': True,
                'enable_ir': False,
                
                # Lowest resolution to ensure it can open even on USB 2.0
                'depth_width': 480,
                'depth_height': 270,
                'depth_fps': 15,
                'depth_format': 'z16'
            }],
            output='screen'
        )
    ])
