"""Launch the OAK-D 1080p HW-encode streamer (depthai_udp_streamer).

This is the fallback profile: OAK-D on-device VideoEncoder at CBR 8 Mbps.
The OAK-D USB device is driven by exactly one DepthAI pipeline at a time, so
this launch is mutually exclusive with oakd_720p_stream_launch.py.
"""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dest_ip_arg = DeclareLaunchArgument('dest_ip', default_value='192.168.0.241')
    base_port_arg = DeclareLaunchArgument('base_port', default_value='9000')
    fps_arg = DeclareLaunchArgument('fps', default_value='30')
    use_h264_arg = DeclareLaunchArgument(
        'use_h264',
        default_value='true',
        description='OAK-D codec: true=H264 Baseline (default), false=MJPEG'
    )

    return LaunchDescription([
        dest_ip_arg,
        base_port_arg,
        fps_arg,
        use_h264_arg,
        Node(
            package='ffw_depthai',
            executable='depthai_udp_streamer',
            name='depthai_udp_streamer',
            output='screen',
            # Node keeps these args for interface parity; the fallback itself
            # hardcodes 192.168.0.241 / 9100 / 9300 and reads use_h264 as a param.
            arguments=[
                LaunchConfiguration('dest_ip'),
                LaunchConfiguration('base_port'),
                LaunchConfiguration('fps'),
            ],
            parameters=[{'use_h264': LaunchConfiguration('use_h264')}]
        )
    ])
