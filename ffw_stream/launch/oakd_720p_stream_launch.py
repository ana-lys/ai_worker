"""Launch the OAK-D 720p raw -> host-CPU x264 streamer (depthai_720p_udp_streamer).

Raw NV12 720p frames come off USB and the host encodes them with x264enc
(veryfast/zerolatency) at a higher bitrate (~20 Mbps) than the OAK-D HW
encoder's 8 Mbps CBR ceiling. Mutually exclusive with oakd_stream_launch.py
because the OAK-D USB device only supports one DepthAI pipeline at a time.
"""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dest_ip_arg = DeclareLaunchArgument('dest_ip', default_value='192.168.0.241')
    video_port_arg = DeclareLaunchArgument('video_port', default_value='9110')
    fps_arg = DeclareLaunchArgument('fps', default_value='30')
    bitrate_arg = DeclareLaunchArgument(
        'bitrate_kbps',
        default_value='20000',
        description='Host x264enc bitrate in kbps (20000 = 20 Mbps)'
    )

    return LaunchDescription([
        dest_ip_arg,
        video_port_arg,
        fps_arg,
        bitrate_arg,
        Node(
            package='ffw_depthai',
            executable='depthai_720p_udp_streamer',
            name='depthai_720p_udp_streamer',
            output='screen',
            arguments=[
                LaunchConfiguration('dest_ip'),
                LaunchConfiguration('video_port'),
                LaunchConfiguration('fps'),
                LaunchConfiguration('bitrate_kbps'),
            ]
        )
    ])
