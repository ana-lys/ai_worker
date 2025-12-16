from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ffw_teleop',
            executable='keyboard_swerve_drive',
            name='keyboard_swerve_drive',
            output='screen',
            parameters=[{
                'topic': '/cmd_vel',
                'update_rate': 100.0,
                'max_vx': 1.0,
                'min_vx': -0.5,
                'max_wz': 1.5,
                'min_wz': -1.5,
                'accel_rate': 2.0,
                'accel_rate_angular': 3.0,
                'vx_discount': 0.02,
                'wz_discount': 0.03
            }]
        )
    ])
