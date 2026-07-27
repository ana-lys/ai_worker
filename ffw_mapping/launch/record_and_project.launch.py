import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Arguments ──────────────────────────────────────────────────────
    start_recording_arg = DeclareLaunchArgument(
        'start_recording', default_value='true',
        description='Whether to start rosbag recording immediately')

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='Laser scan topic for ScanProjector')

    ekf_odom_topic_arg = DeclareLaunchArgument(
        'ekf_odom_topic', default_value='/ekf_odom',
        description='EKF odometry topic for ScanProjector')

    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='/map_scan',
        description='Output PointCloud2 topic')

    start_recording = LaunchConfiguration('start_recording')
    scan_topic = LaunchConfiguration('scan_topic')
    ekf_odom_topic = LaunchConfiguration('ekf_odom_topic')
    output_topic = LaunchConfiguration('output_topic')

    # ── ScanProjector node ────────────────────────────────────────────
    scan_projector_node = Node(
        package='ffw_odom',
        executable='scan_projector',
        name='scan_projector',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'ekf_odom_topic': ekf_odom_topic,
            'output_topic': output_topic,
            'base_frame': 'base_link',
            'map_frame': 'map',
        }]
    )

    # ── Record launch (included, with condition) ──────────────────────
    record_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_mapping'),
                'launch',
                'record_map.launch.py',
            ])
        ),
        condition=IfCondition(start_recording),
    )

    return LaunchDescription([
        start_recording_arg,
        scan_topic_arg,
        ekf_odom_topic_arg,
        output_topic_arg,
        scan_projector_node,
        record_launch,
    ])
