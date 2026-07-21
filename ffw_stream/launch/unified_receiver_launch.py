from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_source', default_value='oakd_lite',
                              description='RGB source: zedm, d435, or oakd_lite'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('oakd_codec', default_value='h264',
                              description='OAK-D codec: h264 (default, low-latency) or mjpeg (fallback)'),
        # Unified receiver: auto-opens the known ports and displays whatever streams are present.
        Node(
            package='ffw_stream',
            executable='realsense_udp_receiver',
            name='unified_dashboard_receiver',
            namespace='cameras',
            parameters=[{
                'headless': LaunchConfiguration('headless'),
                'rgb_source': LaunchConfiguration('rgb_source'),
                'oakd_codec': LaunchConfiguration('oakd_codec'),
            }],
            output='screen'
        )
    ])
