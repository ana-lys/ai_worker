import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('ffw_stream')

    dest_ip_arg = DeclareLaunchArgument('dest_ip', default_value='192.168.0.241')
    base_port_arg = DeclareLaunchArgument('base_port', default_value='9000')
    fps_arg = DeclareLaunchArgument('fps', default_value='30')

    # RealSense Streamer (Handles all RealSense cameras automatically)
    rs_exec = os.path.join(get_package_prefix('ffw_stream'), 'lib', 'ffw_stream', 'realsense_udp_streamer')
    realsense_streamer = ExecuteProcess(
        cmd=[
            rs_exec,
            LaunchConfiguration('dest_ip'),
            LaunchConfiguration('base_port')
        ],
        output='screen'
    )

    return LaunchDescription([
        dest_ip_arg,
        base_port_arg,
        fps_arg,
        realsense_streamer
    ])
