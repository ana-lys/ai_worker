import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
        parameters=[left_config],
        output='screen'
    )

    # Node for Right Camera
    right_node = Node(
        package='ffw_stream',
        executable='realsense_udp_streamer',
        name='camera_right_streamer',
        parameters=[right_config],
        output='screen'
    )

    return LaunchDescription([
        left_node,
        right_node
    ])
