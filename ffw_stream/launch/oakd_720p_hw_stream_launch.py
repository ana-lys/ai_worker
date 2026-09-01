"""Launch the OAK-D 720p on-device HW-encode streamer (depthai_720p_hw_streamer).

Default: native 720p @ 30 fps MJPEG (intra-only, zero-latency). Pass codec:=h264
(and fps:=20 for the legacy 720p20 cadence) for the H264-Baseline zero-lag profile
(no B-frames, IDR every 1s, CBR). USB only carries the encoded bitstream instead
of raw NV12. Mutually exclusive with the other OAK-D profiles — the OAK-D USB
device supports one DepthAI pipeline at a time.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dest_ip_arg = DeclareLaunchArgument('dest_ip', default_value='192.168.0.241')
    video_port_arg = DeclareLaunchArgument('video_port', default_value='9110')
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Stream framerate (native 720p @ 30 fps default)'
    )
    bitrate_arg = DeclareLaunchArgument(
        'bitrate_kbps',
        default_value='8000',
        description='On-device H264 CBR bitrate in kbps (H264 mode only; 8000 = 8 Mbps, same budget as the 1080p fallback)'
    )
    codec_arg = DeclareLaunchArgument(
        'codec',
        default_value='mjpeg',
        description='Stream codec: mjpeg (default, intra-only zero-latency) or h264 (H264-Baseline zero-lag)'
    )

    return LaunchDescription([
        dest_ip_arg,
        video_port_arg,
        fps_arg,
        bitrate_arg,
        codec_arg,
        Node(
            package='ffw_depthai',
            executable='depthai_720p_hw_streamer',
            name='depthai_720p_hw_streamer',
            output='screen',
            arguments=[
                LaunchConfiguration('dest_ip'),
                LaunchConfiguration('video_port'),
                LaunchConfiguration('fps'),
                LaunchConfiguration('bitrate_kbps'),
                LaunchConfiguration('codec'),
            ]
        )
    ])
