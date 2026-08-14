import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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
        default_value='d435',
        description='RGB camera source: d435, zedm, or oakd_lite'
    )
    use_h264_arg = DeclareLaunchArgument(
        'use_h264',
        default_value='true',
        description='OAK-D codec: true=H264 Baseline (default), false=MJPEG'
    )
    color_exposure_arg = DeclareLaunchArgument(
        'color_exposure',
        default_value='0',
        description='Fixed color exposure (us) for the D405 RGB stream; '
                    '0 = keep SDK default auto-exposure'
    )

    def launch_setup(context, *args, **kwargs):
        dest_ip = LaunchConfiguration('dest_ip').perform(context)
        base_port = LaunchConfiguration('base_port').perform(context)
        fps = LaunchConfiguration('fps').perform(context)
        enable_d405s = LaunchConfiguration('enable_d405s').perform(context).lower() == 'true'
        rgb_source = LaunchConfiguration('rgb_source').perform(context)
        color_exposure = LaunchConfiguration('color_exposure').perform(context)
        actions = []

        # RealSense streamer can be used for D405s and/or D435 RGB.
        if enable_d405s or rgb_source == 'd435':
            rs_exec = os.path.join(get_package_prefix('ffw_stream'), 'lib', 'ffw_stream', 'realsense_udp_streamer')
            d405_flag = '--enable-d405s' if enable_d405s else '--disable-d405s'
            d435_flag = '--d435-rgb' if rgb_source == 'd435' else '--no-d435-rgb'
            cmd = [rs_exec, dest_ip, base_port, d405_flag, d435_flag]
            if color_exposure != '0':
                cmd += ['--color-exposure', color_exposure]
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
            actions.append(
                Node(
                    package='ffw_depthai',
                    executable='depthai_udp_streamer',
                    name='depthai_udp_streamer',
                    output='screen',
                    arguments=[dest_ip, base_port, fps],
                    parameters=[{'use_h264': LaunchConfiguration('use_h264')}]
                )
            )
        elif rgb_source != 'd435':
            raise RuntimeError(
                f"Unsupported rgb_source '{rgb_source}'. Use d435, zedm, or oakd_lite."
            )

        return actions

    return LaunchDescription([
        dest_ip_arg,
        base_port_arg,
        fps_arg,
        enable_d405s_arg,
        rgb_source_arg,
        use_h264_arg,
        color_exposure_arg,
        OpaqueFunction(function=launch_setup)
    ])
