"""Launch a joy_node for the Logitech Extreme 3D Pro + the explorer script.

Auto-detects the Logitech by vendor/product/name, starts the driver, and
runs the live input explorer so you can verify every axis and button.

Usage:
    ros2 launch ffw_spacemouse explore_logitech.launch.py
"""

import glob
import os
import re
import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


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
            # udevadm info -a outputs ATTRS{id/vendor} and ATTRS{id/product}
            # (with a slash, NOT camelCase)
            if (f'ATTRS{{id/vendor}}=="{LOGITECH_VENDOR}"' in out and
                f'ATTRS{{id/product}}=="{LOGITECH_PRODUCT}"' in out):
                # Also check name match for Extreme 3D
                if any('ATTRS{name}==' in line and LOGITECH_NAME.lower() in line.lower()
                       for line in out.splitlines()):
                    # Extract the numeric index from /dev/input/jsN
                    m = re.search(r'js(\d+)$', js)
                    if m:
                        return int(m.group(1))
        except Exception:
            continue
    return None


def launch_setup(context):
    logitech_id = _detect_logitech()
    if logitech_id is None:
        # Fallback: try SDL indices 2, 1, 0 (most likely device ids)
        for fallback_id in [2, 1, 0]:
            fallback_path = f'/dev/input/js{fallback_id}'
            if os.path.exists(fallback_path):
                logitech_id = fallback_id
                break

    if logitech_id is not None:
        print(f'[explore_logitech] Logitech Extreme 3D Pro detected as '
              f'device_id={logitech_id}  (/dev/input/js{logitech_id})')
    else:
        logitech_id = 0
        print(f'[explore_logitech] WARNING: Logitech not found, '
              f'defaulting to device_id=0 (/dev/input/js0)')

    return [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_logitech',
            parameters=[{
                'device_id': logitech_id,   # Jazzy SDL-based joy_node uses int index
                'deadzone': 0.0,
                'autorepeat_rate': 100.0,
            }],
            remappings=[('/joy', '/joy_logitech')],
        ),
        Node(
            package='ffw_spacemouse',
            executable='explore_logitech.py',
            name='logitech_explorer',
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
