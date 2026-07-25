from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import glob
import os
import subprocess


def _find_device(vendor, product, name=None):
    """Return /dev/input/jsX matching the given criteria, or None."""
    for sym in ['/dev/input/spacemouse_right', '/dev/input/spacemouse_left',
                '/dev/input/logitech_3dpro']:
        if os.path.exists(sym):
            try:
                out = subprocess.check_output(
                    ['udevadm', 'info', '-a', '-n', os.path.realpath(sym)],
                    text=True, stderr=subprocess.DEVNULL)
                if vendor and f'ATTRS{{idVendor}}=="{vendor}"' in out:
                    return os.path.realpath(sym)
            except Exception:
                continue

    for js in sorted(glob.glob('/dev/input/js*')):
        try:
            out = subprocess.check_output(
                ['udevadm', 'info', '-a', '-n', js],
                text=True, stderr=subprocess.DEVNULL)
            ok = True
            if vendor and f'ATTRS{{idVendor}}=="{vendor}"' not in out:
                ok = False
            if product and f'ATTRS{{idProduct}}=="{product}"' not in out:
                ok = False
            if name:
                if not any('ATTRS{name}==' in line and name.lower() in line.lower()
                           for line in out.splitlines()):
                    ok = False
            if ok:
                return js
        except Exception:
            continue
    return None


def _resolve_devices(context):
    target = LaunchConfiguration('target_arm').perform(context)
    did = LaunchConfiguration('device_id').perform(context)

    # Always SpaceMouse for arm EE control (Logitech handles base/head via base_teleop)
    dev = _find_device('256f', 'c63a')

    if dev is None and did.isdigit():
        dev = f'/dev/input/js{did}'
    elif dev is None:
        dev = f'/dev/input/js{did}' if did else '/dev/input/js0'

    print(f'joy_mapper [target={target}]: using {dev}')
    return [dev]


def generate_launch_description():
    target_arm_arg = DeclareLaunchArgument(
        'target_arm', default_value='right',
        description='Target arm: left or right')
    device_id_arg = DeclareLaunchArgument(
        'device_id', default_value='',
        description='Fallback device ID (ignored if auto-detected)')

    def launch_setup(context):
        dev_paths = _resolve_devices(context)
        dev = dev_paths[0]

        return [
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                namespace=LaunchConfiguration('target_arm').perform(context),
                parameters=[{
                    'dev': dev,
                    'deadzone': 0.0,
                    'autorepeat_rate': 100.0,
                }]
            ),
            Node(
                package='ffw_joy',
                executable='joy_mapper',
                name='joy_mapper',
                namespace=LaunchConfiguration('target_arm').perform(context),
                output='screen',
                parameters=[{
                    'target_arm': LaunchConfiguration('target_arm').perform(context),
                    'publish_rate_hz': 100.0,
                    'reference_frame': 'global',
                    'ee_orientation_slack_deg': 1.0,
                }]
            ),
        ]

    return LaunchDescription([
        target_arm_arg, device_id_arg,
        OpaqueFunction(function=launch_setup),
    ])
