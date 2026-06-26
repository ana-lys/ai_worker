import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('ffw_stream')

    dest_ip_arg = DeclareLaunchArgument('dest_ip', default_value='192.168.0.241')
    base_port_arg = DeclareLaunchArgument('base_port', default_value='9000')
    fps_arg = DeclareLaunchArgument('fps', default_value='30')

    left_config = os.path.join(pkg_dir, 'config', 'camera_left_config.yaml')
    right_config = os.path.join(pkg_dir, 'config', 'camera_right_config.yaml')

    # Left RealSense (Ports: base_port, base_port+1)
    left_streamer = Node(
        package='ffw_stream',
        executable='realsense_udp_streamer',
        name='camera_left_streamer',
        parameters=[left_config],
        output='screen'
    )

    # Right RealSense (Ports: base_port+2, base_port+3)
    right_streamer = Node(
        package='ffw_stream',
        executable='realsense_udp_streamer',
        name='camera_right_streamer',
        parameters=[right_config],
        output='screen'
    )

    # ZED Streamer (Port: base_port+100)
    zed_exec = os.path.join(get_package_prefix('ffw_stream'), 'lib', 'ffw_stream', 'zed_udp_streamer')
    zed_streamer = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'NVJPEG=$(find /usr/lib/aarch64-linux-gnu -name libnvjpeg.so | head -n 1); '
            'if [ -z "$NVJPEG" ]; then NVJPEG=$(find /usr/lib/aarch64-linux-gnu/nvidia -name "libjpeg.so*" | head -n 1); fi; '
            'if [ -z "$NVJPEG" ]; then NVJPEG=$(find /usr/lib/aarch64-linux-gnu/tegra -name "libjpeg.so*" | head -n 1); fi; '
            'export LD_PRELOAD=$NVJPEG; "$0" "$1" "$2" "$3"',
            zed_exec,
            LaunchConfiguration('dest_ip'),
            LaunchConfiguration('base_port'),
            LaunchConfiguration('fps')
        ],
        output='screen'
    )

    return LaunchDescription([
        dest_ip_arg,
        base_port_arg,
        fps_arg,
        left_streamer,
        right_streamer,
        zed_streamer
    ])
