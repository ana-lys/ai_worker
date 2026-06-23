import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from ament_index_python.packages import get_package_share_directory

def get_joint_limits_from_urdf():
    defaults = {
        'arm_lower_limits': [-3.14, -3.14, -3.14, -2.9361, -3.14, -1.57, -1.5804, 0.0],
        'arm_upper_limits': [3.14, 0.1, 3.14, 1.0786, 3.14, 1.57, 1.8201, 1.1],
    }
    return defaults # Simplified for now

def generate_launch_description():
    joint_limits = get_joint_limits_from_urdf()
    
    target_arm_arg = DeclareLaunchArgument('target_arm', default_value='right', description='Target arm: left or right')
    device_id_arg = DeclareLaunchArgument('device_id', default_value='0', description='Device ID integer (e.g., 0 for /dev/input/js0)')

    return LaunchDescription([
        target_arm_arg,
        device_id_arg,
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=LaunchConfiguration('target_arm'),
            parameters=[{
                'device_id': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('device_id'), value_type=int),
                'deadzone': 0.0,
                'autorepeat_rate': 100.0,
            }]
        ),
        Node(
            package='ffw_spacemouse',
            executable='spacemouse_mapper',
            name='spacemouse_mapper',
            namespace=LaunchConfiguration('target_arm'),
            output='screen',
            parameters=[{
                'target_arm': LaunchConfiguration('target_arm'),
                'pos_step': 0.005,
                'rot_step': 0.025,
                'publish_rate_hz': 100.0,
                'trans_sensitivity': 1.5,
                'rot_sensitivity': 1.0,
                'reference_frame': 'global',
                'axis_x': 1,
                'axis_y': 0,
                'axis_z': 2,
                'axis_roll': 4,
                'axis_pitch': 3,
                'axis_yaw': 5,
                'invert_x': False,
                'invert_y': False,
                'invert_z': False,
                'invert_roll': False,
                'invert_pitch': False,
                'invert_yaw': False,
                **joint_limits,
            }]
        ),
    ])
