from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_source', default_value='oakd_lite',
                              description='RGB source: zedm, d435, oakd_lite, oakd_lite_720p, oakd_lite_720p_hw, or oakd_lite_720p_mjpeg'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('oakd_codec', default_value='h264',
                              description='OAK-D codec: h264 (default, low-latency) or mjpeg (fallback)'),
        DeclareLaunchArgument('rs_codec', default_value='h264',
                              description='RealSense D405/D435 codec: h264 (default) or mjpeg (zero-latency)'),
        DeclareLaunchArgument('oakd_720p_video_port', default_value='9110',
                              description='OAK-D 720p stream video port (telemetry = +200)'),
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
                'rs_codec': LaunchConfiguration('rs_codec'),
                'oakd_720p_video_port': LaunchConfiguration('oakd_720p_video_port'),
            }],
            output='screen'
        )
    ])
