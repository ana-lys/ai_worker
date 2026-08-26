import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    dest_ip_arg = DeclareLaunchArgument('dest_ip', default_value='192.168.0.241')
    base_port_arg = DeclareLaunchArgument('base_port', default_value='9000')
    fps_arg = DeclareLaunchArgument('fps', default_value='30')
    enable_d405s_arg = DeclareLaunchArgument(
        'enable_d405s',
        default_value='true',
        description='Enable the D405 hand cameras'
    )
    rgb_source_arg = DeclareLaunchArgument(
        'rgb_source',
        default_value='oakd_lite',
        description='RGB camera source: d435, zedm, oakd_lite, or oakd_lite_720p'
    )
    use_h264_arg = DeclareLaunchArgument(
        'use_h264',
        default_value='true',
        description='OAK-D codec (fallback only): true=H264 Baseline (default), false=MJPEG'
    )
    oakd_video_port_arg = DeclareLaunchArgument(
        'oakd_video_port',
        default_value='9110',
        description='OAK-D 720p stream video port (fallback uses 9100)'
    )
    oakd_bitrate_arg = DeclareLaunchArgument(
        'oakd_bitrate_kbps',
        default_value='20000',
        description='OAK-D 720p host x264enc bitrate in kbps'
    )
    color_exposure_arg = DeclareLaunchArgument(
        'color_exposure',
        default_value='0',
        description='Fixed color exposure (us) for the D405 RGB stream; '
                    '0 = keep SDK default auto-exposure'
    )
    color_wb_arg = DeclareLaunchArgument(
        'color_wb',
        default_value='0',
        description='Fixed manual white balance (K) for the D405 RGB stream; '
                    '0 = leave SDK default white balance'
    )

    def launch_setup(context, *args, **kwargs):
        dest_ip = LaunchConfiguration('dest_ip').perform(context)
        base_port = LaunchConfiguration('base_port').perform(context)
        fps = LaunchConfiguration('fps').perform(context)
        enable_d405s = LaunchConfiguration('enable_d405s').perform(context).lower() == 'true'
        rgb_source = LaunchConfiguration('rgb_source').perform(context)
        color_exposure = LaunchConfiguration('color_exposure').perform(context)
        color_wb = LaunchConfiguration('color_wb').perform(context)
        oakd_video_port = LaunchConfiguration('oakd_video_port').perform(context)
        oakd_bitrate = LaunchConfiguration('oakd_bitrate_kbps').perform(context)
        actions = []

        # RealSense streamer can be used for D405s and/or D435 RGB.
        if enable_d405s or rgb_source == 'd435':
            rs_exec = os.path.join(get_package_prefix('ffw_stream'), 'lib', 'ffw_stream', 'realsense_udp_streamer')
            d405_flag = '--enable-d405s' if enable_d405s else '--disable-d405s'
            d435_flag = '--d435-rgb' if rgb_source == 'd435' else '--no-d435-rgb'
            cmd = [rs_exec, dest_ip, base_port, d405_flag, d435_flag]
            if color_exposure != '0':
                cmd += ['--color-exposure', color_exposure]
            if color_wb != '0':
                cmd += ['--color-wb', color_wb]
            actions.append(
                ExecuteProcess(cmd=cmd, output='screen')
            )

        if rgb_source == 'zedm':
            zed_exec = os.path.join(get_package_prefix('ffw_stream'), 'lib', 'ffw_stream', 'zed_udp_streamer')
            actions.append(
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'NVJPEG=$(find /usr/lib/aarch64-linux-gnu -name libnvjpeg.so | head -n 1); '
                        'if [ -z "$NVJPEG" ]; then NVJPEG=$(find /usr/lib/aarch64-linux-gnu/nvidia -name "libjpeg.so*" | head -n 1); fi; '
                        'if [ -z "$NVJPEG" ]; then NVJPEG=$(find /usr/lib/aarch64-linux-gnu/tegra -name "libjpeg.so*" | head -n 1); fi; '
                        'echo "[ZED] Preloading proprietary NVIDIA JPEG library: $NVJPEG"; '
                        'export LD_PRELOAD=$NVJPEG; "$0" "$1" "$2" "$3"',
                        zed_exec,
                        dest_ip,
                        base_port,
                        fps
                    ],
                    output='screen'
                )
            )
        elif rgb_source == 'oakd_lite':
            # Fallback: OAK-D 1080p HW-encode (CBR 8 Mbps) via its own launch file
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('ffw_stream'),
                            'launch', 'oakd_stream_launch.py'
                        )
                    ),
                    launch_arguments={
                        'dest_ip': dest_ip,
                        'base_port': base_port,
                        'fps': fps,
                        'use_h264': LaunchConfiguration('use_h264'),
                    }.items()
                )
            )
        elif rgb_source == 'oakd_lite_720p':
            # New: OAK-D 720p raw -> host-CPU x264 (~20 Mbps) via its own launch file
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('ffw_stream'),
                            'launch', 'oakd_720p_stream_launch.py'
                        )
                    ),
                    launch_arguments={
                        'dest_ip': dest_ip,
                        'video_port': oakd_video_port,
                        'fps': fps,
                        'bitrate_kbps': oakd_bitrate,
                    }.items()
                )
            )
        elif rgb_source != 'd435':
            raise RuntimeError(
                f"Unsupported rgb_source '{rgb_source}'. "
                "Use d435, zedm, oakd_lite, or oakd_lite_720p."
            )

        return actions

    return LaunchDescription([
        dest_ip_arg,
        base_port_arg,
        fps_arg,
        enable_d405s_arg,
        rgb_source_arg,
        use_h264_arg,
        oakd_video_port_arg,
        oakd_bitrate_arg,
        color_exposure_arg,
        color_wb_arg,
        OpaqueFunction(function=launch_setup)
    ])
