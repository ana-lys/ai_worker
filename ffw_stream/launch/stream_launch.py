import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    target_ip_arg = DeclareLaunchArgument(
        'target_ip',
        default_value='192.168.0.241',
        description='Target IP for UDP streaming'
    )
    target_ip = LaunchConfiguration('target_ip')

    config_dir = os.path.join(
        get_package_share_directory('ffw_stream'),
        'config'
    )

    left_config = os.path.join(config_dir, 'camera_left_config.yaml')
    right_config = os.path.join(config_dir, 'camera_right_config.yaml')

    left_node = Node(
        package='ffw_stream',
        executable='realsense_udp_streamer',
        name='camera_left_streamer',
        parameters=[left_config, {'target_ip': target_ip}],
        output='screen'
    )

    # Delay right camera by 3 seconds to avoid USB enumeration race
    # with left camera on shared USB 2.0 bus
    right_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='ffw_stream',
            executable='realsense_udp_streamer',
            name='camera_right_streamer',
            parameters=[right_config, {'target_ip': target_ip}],
            output='screen'
        )]
    )

    return LaunchDescription([
        target_ip_arg,
        left_node,
        right_node,
    ])
