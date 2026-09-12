from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    source_frame_arg = DeclareLaunchArgument(
        'source_frame',
        default_value='base_link',
        description='Frame the raw /oakd/marker_board_pose is expressed in'
    )

    child_frame_arg = DeclareLaunchArgument(
        'child_frame',
        default_value='marker_frame',
        description='TF frame name broadcast by the marker EKF (base_link\'s '
                     'world-anchor frame for the teleop global limit feature)'
    )

    board_yaw_offset_arg = DeclareLaunchArgument(
        'board_yaw_offset_rad',
        default_value='1.5707963267948966',
        description='Fixed yaw correction (rad) about the marker\'s own Z axis, '
                     'applied so marker_frame reads ROS front/left/up'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/oakd/marker_board_pose',
        description='Input PoseStamped topic for the detected marker board pose'
    )

    # marker_pose_corrector: inverts + yaw-corrects the raw ~4-5 Hz marker
    # detection into an absolute base_link-in-marker_frame pose measurement.
    marker_pose_corrector_node = Node(
        package='ffw_odom',
        executable='marker_pose_corrector',
        name='marker_pose_corrector',
        output='screen',
        parameters=[{
            'source_frame': LaunchConfiguration('source_frame'),
            'child_frame': LaunchConfiguration('child_frame'),
            'board_yaw_offset_rad': LaunchConfiguration('board_yaw_offset_rad'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': '/oakd/marker_frame_baselink_pose',
        }]
    )

    # robot_localization ekf_node: fuses continuous /odom with the low-rate
    # marker correction above into a smooth, high-rate marker_frame <->
    # base_link TF (publish_tf: true in marker_ekf.yaml).
    marker_ekf_config = PathJoinSubstitution([
        FindPackageShare('ffw_odom'),
        'config',
        'marker_ekf.yaml'
    ])

    marker_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='marker_ekf_filter_node',
        output='screen',
        parameters=[marker_ekf_config],
        remappings=[
            ('odometry/filtered', 'marker_ekf_odom')
        ],
    )

    return LaunchDescription([
        source_frame_arg,
        child_frame_arg,
        board_yaw_offset_arg,
        input_topic_arg,
        marker_pose_corrector_node,
        marker_ekf_node,
    ])
