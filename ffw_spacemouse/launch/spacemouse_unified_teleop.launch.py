import glob
import os
import re
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def _detect_spacemice():
    """Auto-detect two SpaceMouse devices and return their SDL indices as (left, right).

    Scans /dev/input/js* for 3Dconnexion PowerMouse / SpaceMouse (vendor 256f,
    product c63a), then sorts by physical USB path so the assignment is
    deterministic regardless of device-discovery order.

    Returns (left_sdl_id, right_sdl_id) as strings, or (None, None) if fewer
    than 2 SpaceMice are found.
    """
    spacemice = []  # list of (js_path, usb_phys_path)

    for js in sorted(glob.glob('/dev/input/js*')):
        try:
            out = subprocess.check_output(
                ['udevadm', 'info', '-a', '-n', js],
                text=True, stderr=subprocess.DEVNULL)
            if ('ATTRS{idVendor}=="256f"' in out and
                'ATTRS{idProduct}=="c63a"' in out):
                spacemice.append(js)
        except Exception:
            continue

    if len(spacemice) < 2:
        print(f'[spacemouse_unified_teleop] WARNING: Expected 2 SpaceMice, '
              f'found {len(spacemice)}. Falling back to device_id 0 and 1.')
        return None, None

    # Resolve physical USB path for deterministic left/right assignment
    # Build a map from /dev/input/jsN → USB port path via by-path symlinks
    js_to_usbpath = {}
    for symlink in glob.glob('/dev/input/by-path/*-joystick'):
        if '-event-' in symlink:
            continue
        try:
            real = os.path.realpath(symlink)
            if real in spacemice:
                js_to_usbpath[real] = symlink
        except Exception:
            continue

    if len(js_to_usbpath) >= 2:
        # Sort by USB path string for deterministic assignment
        sorted_usb = sorted(js_to_usbpath.items(), key=lambda x: x[1])
        left_js = sorted_usb[0][0]
        right_js = sorted_usb[1][0]
    else:
        # Fall back to /dev/input/js* alphabetical order
        left_js = spacemice[0]
        right_js = spacemice[1]
        print(f'[spacemouse_unified_teleop] WARNING: Could not resolve USB '
              f'paths; falling back to js* order for left/right assignment.')

    # Extract SDL index from /dev/input/jsN
    def _sdl_index(path):
        m = re.search(r'js(\d+)$', path)
        return m.group(1) if m else '0'

    left_id = _sdl_index(left_js)
    right_id = _sdl_index(right_js)

    left_usb = js_to_usbpath.get(left_js, left_js)
    right_usb = js_to_usbpath.get(right_js, right_js)

    print(f'[spacemouse_unified_teleop] Auto-detected SpaceMice:')
    print(f'  LEFT  arm ← SDL {left_id}  ({left_usb})')
    print(f'  RIGHT arm ← SDL {right_id} ({right_usb})')

    return left_id, right_id


def launch_setup(context):
    ffw_spacemouse_dir = get_package_share_directory('ffw_spacemouse')
    mapper_launch_file = os.path.join(ffw_spacemouse_dir, 'launch', 'spacemouse_mapper.launch.py')

    hardware_mode_str = LaunchConfiguration('hardware_mode').perform(context)
    hardware_mode = hardware_mode_str.lower() in ('true', '1', 'yes')
    robot_model = LaunchConfiguration('robot_model').perform(context)

    # Resolve device IDs — auto-detect if left at defaults
    left_id = LaunchConfiguration('left_device_id').perform(context)
    right_id = LaunchConfiguration('right_device_id').perform(context)

    # Logitech joystick override
    use_logitech_str = LaunchConfiguration('use_logitech').perform(context)
    use_logitech = use_logitech_str.lower() in ('true', '1', 'yes')
    logitech_device_id = LaunchConfiguration('logitech_device_id').perform(context)

    # Quest controller ARM override
    use_quest_str = LaunchConfiguration('use_quest').perform(context)
    use_quest = use_quest_str.lower() in ('true', '1', 'yes')
    quest_device = LaunchConfiguration('quest_device').perform(context)

    if left_id == '0' and right_id == '1':
        auto_left, auto_right = _detect_spacemice()
        if auto_left is not None:
            left_id = auto_left
            right_id = auto_right

    nodes = []

    # Left SpaceMouse Mapper (includes joy_node)
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapper_launch_file),
            launch_arguments={
                'target_arm': 'left',
                'device_id': left_id,
            }.items()
        )
    )

    # Right SpaceMouse Mapper (includes joy_node) — quest_publish_notifications
    # false: only the left instance fires /ffw_control/notification; both still
    # publish their own /quest/<arm>/active (quest_teleop_plan §7).
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapper_launch_file),
            launch_arguments={
                'target_arm': 'right',
                'device_id': right_id,
                'quest_publish_notifications': 'false',
            }.items()
        )
    )

    # Base Teleop Node
    base_teleop_node = Node(
        package='ffw_spacemouse',
        executable='spacemouse_base_teleop',
        name='spacemouse_base_teleop',
        output='screen',
        parameters=[{
            'base_joy_topic': '/right/joy',
            'aux_joy_topic':  '/left/joy',
            'head_topic': '/leader/joystick_controller_left/joint_trajectory',
            'max_linear_vel': 0.6,
            'max_angular_vel': 0.5,
            'head_step': 0.03125,
            'axis_x': 1,
            'axis_y': 0,
            'axis_yaw': 5,
            'axis_z': 2,
            'axis_pitch': 3,
            'axis_head_pan': 5,
            'invert_x': False,
            'invert_y': False,
            'invert_yaw': False,
            'invert_z': False,
            'invert_pitch': False,
            'invert_head_pan': False,
            'hardware_mode': hardware_mode,
        }]
    )
    nodes.append(base_teleop_node)

    # IK Solver Teleop Node
    nodes.append(
        Node(
            package='ffw_collision_checker',
            executable='ffw_ik_solver_teleop',
            name='ffw_ik_solver_teleop',
            output='screen',
            parameters=[{
                'hardware_mode': hardware_mode,
                'robot_model': robot_model,
            }]
        )
    )

    # Logitech Joystick (optional — base control override)
    if use_logitech:
        logitech_launch = os.path.join(
            ffw_spacemouse_dir, 'launch', 'logitech_teleop.launch.py')
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(logitech_launch),
                launch_arguments={
                    'device_id': logitech_device_id,
                    'max_linear_vel': '0.6',
                    'max_angular_vel': '0.5',
                }.items()
            )
        )

    # Quest controller bridge (optional — ARM override). quest_to_ros2.py is the
    # pure telemetry bridge publishing /quest_state (no control logic). Its
    # output goes to the launch log, not the console, so the mapper/base node
    # prints stay readable instead of being flooded by the bridge's 100 Hz HUD
    # (quest_teleop_plan §7). For a live quest HUD, run the script with --tui
    # in its own terminal.
    if use_quest:
        quest_script = os.path.join(
            get_package_prefix('ffw_spacemouse'),
            'lib', 'ffw_spacemouse', 'quest_to_ros2.py')
        nodes.append(
            ExecuteProcess(
                cmd=['python3', quest_script, '--port', quest_device],
                output='log',
            )
        )

    # Failsafe shutdown event — registered against the base_teleop_node
    if hardware_mode:
        nodes.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=base_teleop_node,
                    on_exit=[
                        EmitEvent(event=Shutdown(
                            reason='SpaceMouse Base Teleop terminated due to Joint State Failsafe!'))
                    ]
                )
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_mode', default_value='true',
            description='If true, enables base teleop, mode switching, and hardware/Gazebo sync.'),
        DeclareLaunchArgument(
            'robot_model', default_value='bg2',
            description='Robot model variant: bg2 (RH-P12-RN-A grippers) or smtm (XM430 right gripper).'),
        DeclareLaunchArgument(
            'left_device_id', default_value='0',
            description='SDL joystick index for left SpaceMouse (auto-detected if left at default).'),
        DeclareLaunchArgument(
            'right_device_id', default_value='1',
            description='SDL joystick index for right SpaceMouse (auto-detected if left at default).'),
        DeclareLaunchArgument(
            'use_logitech', default_value='false',
            description='Enable Logitech Extreme 3D Pro as base control override.'),
        DeclareLaunchArgument(
            'logitech_device_id', default_value='-1',
            description='SDL device index for Logitech (auto-detect if -1).'),
        DeclareLaunchArgument(
            'use_quest', default_value='false',
            description='Enable Quest controller ARM override (runs quest_to_ros2.py bridge + forwards quest params to joy_hand).'),
        DeclareLaunchArgument(
            'quest_device', default_value='9500',
            description='Port the Quest stream arrives on (passed to quest_to_ros2.py --port).'),
        OpaqueFunction(function=launch_setup),
    ])
