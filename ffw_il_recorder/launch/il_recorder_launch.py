from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Defaults mirror ffw_stream/unified_receiver_launch.py, so the
        # recorder can drop into the receiver's slot on the same ports.
        DeclareLaunchArgument('rgb_source', default_value='oakd_lite',
                              description='RGB source (informational: head feed is the OAK-D 720p stream)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Disable the recorder live-view window'),
        DeclareLaunchArgument('oakd_codec', default_value='h264',
                              description='Head (OAK-D 720p) codec: h264 (default, H264-Baseline zero-lag) or mjpeg (zero-latency)'),
        DeclareLaunchArgument('rs_codec', default_value='h264',
                              description='Right RGB (D405 cam1) codec: h264 (default) or mjpeg (zero-latency)'),
        DeclareLaunchArgument('oakd_720p_video_port', default_value='9110',
                              description='OAK-D 720p stream video port (telemetry = +200)'),
        DeclareLaunchArgument('right_rgb_port', default_value='9003',
                              description='D405 cam1 "IR" port = right wrist RGB'),
        DeclareLaunchArgument('record_dir', default_value='~/robotis_ws/records',
                              description='Root dir for episode_XXXX/ recordings'),
        DeclareLaunchArgument('jpeg_quality', default_value='95',
                              description='JPEG save quality (1-100)'),
        Node(
            package='ffw_il_recorder',
            executable='il_episode_recorder',
            name='il_episode_recorder',
            output='screen',
            parameters=[{
                'headless': LaunchConfiguration('headless'),
                'rgb_source': LaunchConfiguration('rgb_source'),
                'oakd_codec': LaunchConfiguration('oakd_codec'),
                'rs_codec': LaunchConfiguration('rs_codec'),
                'oakd_720p_video_port': LaunchConfiguration('oakd_720p_video_port'),
                'right_rgb_port': LaunchConfiguration('right_rgb_port'),
                'record_dir': LaunchConfiguration('record_dir'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }],
        )
    ])
