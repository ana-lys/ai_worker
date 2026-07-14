import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    possible_paths = [
        '/home/lys/robotis_ws/src/ai_worker/ffw_mapping/all_walls_downsampled_rotated.txt',
        '/root/ros2_ws/src/ai_worker/ffw_mapping/all_walls_downsampled_rotated.txt',
        '/root/robotis_ws/src/ai_worker/ffw_mapping/all_walls_downsampled_rotated.txt',
    ]
    default_map_path = possible_paths[0]
    for path in possible_paths:
        if os.path.exists(path):
            default_map_path = path
            break

    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path,
        description='Path to the static map CSV/TXT file'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic for the input laser scan'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic for raw odometry'
    )
    
    corrected_odom_topic_arg = DeclareLaunchArgument(
        'corrected_odom_topic',
        default_value='/odom_corrected',
        description='Topic for corrected odometry'
    )
    
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map coordinate frame ID'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry coordinate frame ID'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base link coordinate frame ID'
    )

    # Node configuration
    scan_to_map_icp_node = Node(
        package='ffw_odom',
        executable='scan_to_map_icp',
        name='scan_to_map_icp',
        output='screen',
        parameters=[{
            'map_file': LaunchConfiguration('map_file'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'corrected_odom_topic': LaunchConfiguration('corrected_odom_topic'),
            'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'max_correspondence_dist': 0.4,
            'huber_delta': 0.10,
            'max_iterations': 30,
            'min_inlier_ratio': 0.25,
            'normal_k_neighbors': 8,
            'translation_eps': 1e-4,
            'rotation_eps': 1e-5,
            'max_accepted_rms': 0.10,
            'verbose': False
        }]
    )

    return LaunchDescription([
        map_file_arg,
        scan_topic_arg,
        odom_topic_arg,
        corrected_odom_topic_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        scan_to_map_icp_node
    ])
