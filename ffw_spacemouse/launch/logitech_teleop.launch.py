"""Launch Logitech Extreme 3D Pro base teleop.

Auto-detects the Logitech, starts the driver and the logitech_base_teleop node.

Usage:
    ros2 launch ffw_spacemouse logitech_teleop.launch.py
    ros2 launch ffw_spacemouse logitech_teleop.launch.py device_id:=1
    ros2 launch ffw_spacemouse logitech_teleop.launch.py max_linear_vel:=1.0 max_angular_vel:=0.8
"""

import glob
import os
import re
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions


LOGITECH_VENDOR = '046d'
LOGITECH_PRODUCT = 'c215'
LOGITECH_NAME = 'Extreme 3D'


def _detect_logitech():
    """Return the SDL device_id (int) for the Logitech, or None."""
    for js in sorted(glob.glob('/dev/input/js*')):
        try:
            out = subprocess.check_output(
                ['udevadm', 'info', '-a', '-n', js],
                text=True, stderr=subprocess.DEVNULL)
            if (f'ATTRS{{id/vendor}}=="{LOGITECH_VENDOR}"' in out and
                f'ATTRS{{id/product}}=="{LOGITECH_PRODUCT}"' in out):
                if any('ATTRS{name}==' in line and LOGITECH_NAME.lower() in line.lower()
                       for line in out.splitlines()):
                    m = re.search(r'js(\d+)$', js)
                    if m:
                        return int(m.group(1))
        except Exception:
            continue
    return None


def launch_setup(context):
    device_id_str = LaunchConfiguration('device_id').perform(context)
    device_id = int(device_id_str) if device_id_str != '-1' else _detect_logitech()

    max_lin_str = LaunchConfiguration('max_linear_vel').perform(context)
    max_ang_str = LaunchConfiguration('max_angular_vel').perform(context)
    max_lin = float(max_lin_str)
    max_ang = float(max_ang_str)

    if device_id is not None:
        print(f'[logitech_teleop] Logitech device_id={device_id}', flush=True)
    else:
        device_id = 0
        print(f'[logitech_teleop] WARNING: Logitech not found, defaulting to 0',
              flush=True)

    return [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_logitech',
            parameters=[{
                'device_id': device_id,
                'deadzone': 0.0,
                'autorepeat_rate': 100.0,
            }],
            remappings=[('/joy', '/joy_logitech')],
        ),
        Node(
            package='ffw_spacemouse',
            executable='logitech_base_teleop.py',
            name='logitech_base_teleop',
            output='screen',
            parameters=[{
                'joy_topic': '/joy_logitech',
                'cmd_vel_topic': '/cmd_vel',
                'max_linear_vel': max_lin,
                'max_angular_vel': max_ang,
                'axis_x': 1,
                'axis_y': 0,
                'axis_yaw': 2,
                'axis_speed': 3,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id', default_value='-1',
            description='SDL device index for Logitech (auto-detect if -1).'),
        DeclareLaunchArgument(
            'max_linear_vel', default_value='0.6',
            description='Maximum linear velocity (m/s).'),
        DeclareLaunchArgument(
            'max_angular_vel', default_value='0.5',
            description='Maximum angular velocity (rad/s).'),
        OpaqueFunction(function=launch_setup),
    ])
