import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    dest_ip_arg = DeclareLaunchArgument(
        'dest_ip',
        default_value='192.168.0.241',
        description='Destination IP address for the UDP stream'
    )
    
    base_port_arg = DeclareLaunchArgument(
        'base_port',
        default_value='9000',
        description='Base port for the UDP stream (ZED uses base_port + 100)'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )

    # Resolve absolute path to the executable
    zed_exec = os.path.join(get_package_prefix('ffw_stream'), 'lib', 'ffw_stream', 'zed_udp_streamer')

    # zed_udp_streamer is a pure C++ executable (not a ROS 2 node)
    streamer_cmd = ExecuteProcess(
        cmd=[
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
        streamer_cmd
    ])
