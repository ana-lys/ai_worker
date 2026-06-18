import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    target_ip_arg = DeclareLaunchArgument(
        'target_ip',
        default_value='100.83.72.18',
        description='Target IP for UDP streaming'
    )
    target_ip = LaunchConfiguration('target_ip')

    config_dir = os.path.join(
        get_package_share_directory('ffw_stream'),
        'config'
    )

    # Configuration files for each camera
    left_config = os.path.join(config_dir, 'camera_left_config.yaml')
    right_config = os.path.join(config_dir, 'camera_right_config.yaml')

    # Node for Left Camera
    left_node = Node(
        package='ffw_stream',
        executable='realsense_udp_streamer',
        name='camera_left_streamer',
        parameters=[left_config, {'target_ip': target_ip}],
        output='screen'
    )

    # Node for Right Camera
    right_node = Node(
        package='ffw_stream',
        executable='realsense_udp_streamer',
        name='camera_right_streamer',
        parameters=[right_config, {'target_ip': target_ip}],
        output='screen'
    )

    return LaunchDescription([
        target_ip_arg,
        left_node,
        right_node
    ])
