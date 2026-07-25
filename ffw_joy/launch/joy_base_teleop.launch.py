from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import glob
import os
import subprocess


def _find_device(vendor, product, name=None):
    """Return /dev/input/jsX matching the given criteria, or None."""
    # Check udev symlinks first
    for sym, target_vendor in [('/dev/input/spacemouse_right', '256f'),
                               ('/dev/input/spacemouse_left', '256f'),
                               ('/dev/input/logitech_3dpro', '046d')]:
        if os.path.exists(sym):
            real = os.path.realpath(sym)
            try:
                out = subprocess.check_output(
                    ['udevadm', 'info', '-a', '-n', real],
                    text=True, stderr=subprocess.DEVNULL)
                if vendor and f'ATTRS{{idVendor}}=="{vendor}"' in out:
                    return real
            except Exception:
                continue

    # Scan live js* devices
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
                found = any('ATTRS{name}==' in line and name.lower() in line.lower()
                            for line in out.splitlines())
                if not found:
                    ok = False
            if ok:
                return js
        except Exception:
            continue
    return None


def launch_setup(context):
    right_ns = LaunchConfiguration('right_namespace').perform(context)
    left_ns = LaunchConfiguration('left_namespace').perform(context)
    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    enable_node = LaunchConfiguration('enable_node').perform(context).lower() in ('true', '1', 'yes')

    nodes = []

    # Logitech 3D Pro — identified by unique vendor/product so it's never
    # confused with the SpaceMice (which share the same 256f:c63a ID).
    logitech_dev = _find_device('046d', 'c215', name='Extreme 3D')
    if logitech_dev:
        print(f'joy_base_teleop: Logitech 3D Pro → {logitech_dev}')
    else:
        logitech_dev = '/dev/input/js2'
        print(f'joy_base_teleop: Logitech not found, defaulting to {logitech_dev}')

    nodes.append(Node(
        package='joy', executable='joy_node', name='joy_node_logitech',
        parameters=[{'dev': logitech_dev, 'deadzone': 0.0, 'autorepeat_rate': 100.0}],
    ))

    # Right SpaceMouse — first SpaceMouse found (order in /dev/input/js*)
    right_dev = _find_device('256f', 'c63a', name='SpaceMouse')
    if right_dev:
        print(f'joy_base_teleop: right SpaceMouse → {right_dev}')
    else:
        right_dev = '/dev/input/js0'
        print(f'joy_base_teleop: right SpaceMouse not found, defaulting to {right_dev}')

    nodes.append(Node(
        package='joy', executable='joy_node', name='joy_node_right',
        namespace=right_ns,
        parameters=[{'dev': right_dev, 'deadzone': 0.0, 'autorepeat_rate': 100.0}],
    ))

    # Left SpaceMouse — second SpaceMouse found
    left_dev = _find_device('256f', 'c63a', name='SpaceMouse')
    if left_dev == right_dev:
        # Same device returned; scan manually for the next one
        left_dev = None
        found_right = False
        for js in sorted(glob.glob('/dev/input/js*')):
            if js == right_dev:
                found_right = True
                continue
            if not found_right:
                continue
            try:
                out = subprocess.check_output(
                    ['udevadm', 'info', '-a', '-n', js],
                    text=True, stderr=subprocess.DEVNULL)
                if ('ATTRS{idVendor}=="256f"' in out and
                    'ATTRS{idProduct}=="c63a"' in out):
                    left_dev = js
                    break
            except Exception:
                continue

    if left_dev:
        print(f'joy_base_teleop: left SpaceMouse → {left_dev}')
    else:
        left_dev = '/dev/input/js1'
        print(f'joy_base_teleop: left SpaceMouse not found, defaulting to {left_dev}')

    nodes.append(Node(
        package='joy', executable='joy_node', name='joy_node_left',
        namespace=left_ns,
        parameters=[{'dev': left_dev, 'deadzone': 0.0, 'autorepeat_rate': 100.0}],
    ))

    # Base teleop node (conditionally enabled — unified launcher may create it inline)
    if enable_node:
        nodes.append(Node(
            package='ffw_joy',
            executable='joy_base_teleop',
            name='joy_base_teleop',
            output='screen',
            parameters=[{
                'sim': sim,
                'right_namespace': right_ns,
                'left_namespace': left_ns,
                'right_topic': f'/{right_ns}/joy',
                'left_topic': f'/{left_ns}/joy',
                'base_frame': 'base_link',
                'elevator_frame': 'elevator',
                'head_frame': 'head',
                'max_base_lin_vel': 0.5,
                'max_base_ang_vel': 2.0,
                'max_head_vel': 0.5,
                'max_elevator_vel': 0.3,
                'base_sensitivity': 1.0,
                'head_sensitivity': 1.0,
                'elevator_sensitivity': 1.0,
                'logitech_axis_x': 0,
                'logitech_axis_y': 1,
                'logitech_axis_yaw': 3,
                'logitech_axis_throttle': 2,
                'logitech_axis_hat_x': 4,
                'logitech_axis_hat_y': 5,
                'logitech_hat_speed': 0.5,
                'logitech_throttle_power': 1.0,
                'precision_factor': 0.3,
            }]
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Simulation mode: skip joint_states and snap-back'),
        DeclareLaunchArgument(
            'left_namespace', default_value='left'),
        DeclareLaunchArgument(
            'right_namespace', default_value='right'),
        DeclareLaunchArgument(
            'enable_node', default_value='true',
            description='Launch the joy_base_teleop node itself (vs only device joy_nodes)'),
        OpaqueFunction(function=launch_setup),
    ])
