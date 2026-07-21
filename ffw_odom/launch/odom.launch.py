import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='Enable EKF fusion of laser and swerve odometry'
    )
    
    launch_rf2o_arg = DeclareLaunchArgument(
        'launch_rf2o',
        default_value='true',
        description='Whether to launch rf2o laser odometry'
    )
    
    use_ekf = LaunchConfiguration('use_ekf')
    launch_rf2o = LaunchConfiguration('launch_rf2o')

    # rf2o node
    rf2o_node = Node(
        package='ffw_odom',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        arguments=['--ros-args', '--log-level', 'error'],
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/rf2o/odom',
            'publish_tf' : False,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 30.0
        }],
        condition=IfCondition(launch_rf2o)
    )

    # robot_localization ekf node
    ekf_config = PathJoinSubstitution([
        FindPackageShare('ffw_odom'),
        'config',
        'ekf.yaml'
    ])
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', 'ekf_odom')
        ],
        condition=IfCondition(use_ekf)
    )

    # scan_to_map_icp node
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
    scan_to_map_icp_node = Node(
        package='ffw_odom',
        executable='scan_to_map_icp',
        name='scan_to_map_icp',
        output='screen',
        parameters=[{
            'map_file': default_map_path,
            'scan_topic': '/scan',
            'odom_topic': '/odom',
            'corrected_odom_topic': '/icp_pose_raw',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'max_accepted_rms': 0.15,
            'max_iterations': 50,
            'max_correspondence_dist': 0.15,
            'start_active': True,
            'fallback_hysteresis': 3,
            'scan_to_scan_hysteresis': 2,
            'verbose': False
        }]
    )

    return LaunchDescription([
        use_ekf_arg,
        launch_rf2o_arg,
        rf2o_node,
        ekf_node,
        scan_to_map_icp_node
    ])
