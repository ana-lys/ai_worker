import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions

def get_joint_limits_from_urdf():
    defaults = {
        'lift_lower_limits': [-0.5],
        'lift_upper_limits': [0.0],
        'head_lower_limits': [-0.2317, -0.35],
        'head_upper_limits': [0.6951, 0.35],
    }
    return defaults

def generate_launch_description():
    joint_limits = get_joint_limits_from_urdf()
    
    left_device_id_arg = DeclareLaunchArgument('left_device_id', default_value='0', description='Device ID for Left SpaceMouse (Base)')
    right_device_id_arg = DeclareLaunchArgument('right_device_id', default_value='1', description='Device ID for Right SpaceMouse (Lift/Head)')

    return LaunchDescription([
        left_device_id_arg,
        right_device_id_arg,
        
        # Left SpaceMouse joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_left',
            namespace='spacemouse/left',
            parameters=[{
                'device_id': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('left_device_id'), value_type=int),
                'deadzone': 0.0,
                'autorepeat_rate': 100.0,
            }]
        ),
        
        # Right SpaceMouse joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_right',
            namespace='spacemouse/right',
            parameters=[{
                'device_id': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('right_device_id'), value_type=int),
                'deadzone': 0.0,
                'autorepeat_rate': 100.0,
            }]
        ),

        # Base, Elevator, and Head Teleop Node
        Node(
            package='ffw_spacemouse',
            executable='spacemouse_base_teleop',
            name='spacemouse_base_teleop',
            output='screen',
            parameters=[{
                'left_joy_topic': '/spacemouse/left/joy',
                'right_joy_topic': '/spacemouse/right/joy',
                'max_linear_vel': 1.0,
                'max_angular_vel': 1.5,
                'axis_x': 1,
                'axis_y': 0,
                'axis_yaw': 5,
                'axis_z': 2,
                'axis_pitch': 5,
                'axis_head_pan': 3,
                'invert_x': False,
                'invert_y': False,
                'invert_yaw': False,
                'invert_z': False,
                'invert_pitch': False,
                'invert_head_pan': False,
                'lift_step': 0.01,
                'head_step': 0.05,
                **joint_limits,
            }]
        ),
    ])
