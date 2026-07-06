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
    
    use_ekf = LaunchConfiguration('use_ekf')

    # rf2o node
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/rf2o/odom',
            'publish_tf' : False,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 20.0
        }]
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
            ('odometry/filtered', 'ffw_laser_odom')
        ],
        condition=IfCondition(use_ekf)
    )

    return LaunchDescription([
        use_ekf_arg,
        rf2o_node,
        ekf_node
    ])
