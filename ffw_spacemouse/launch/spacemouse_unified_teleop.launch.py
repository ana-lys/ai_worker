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


SPACEMOUSE_VENDOR = '256f'
SPACEMOUSE_PRODUCT = 'c63a'
LOGITECH_VENDOR = '046d'
LOGITECH_PRODUCT = 'c215'
LOGITECH_NAME = 'Extreme 3D'


def _scan_joysticks():
    """Classify every /dev/input/js* by USB vendor/product.

    Returns a dict {js_path: kind} for recognized devices ('spacemouse' or
    'logitech'). SpaceMice are 3Dconnexion 256f:c63a; the Logitech Extreme 3D
    Pro is 046d:c215.

    udevadm uses camelCase keys (ATTRS{idVendor}), NOT the slash form
    (ATTRS{id/vendor}) — the slash form never matched anything, so a missing
    Logitech silently defaulted to device 0 and stole a SpaceMouse. Detection
    is a single scan so the two never collide even when one is unplugged.
    """
    kinds = {}
    for js in sorted(glob.glob('/dev/input/js*')):
        try:
            out = subprocess.check_output(
                ['udevadm', 'info', '-a', '-n', js],
                text=True, stderr=subprocess.DEVNULL)
        except Exception:
            continue
        if (f'ATTRS{{idVendor}}=="{SPACEMOUSE_VENDOR}"' in out and
            f'ATTRS{{idProduct}}=="{SPACEMOUSE_PRODUCT}"' in out):
            kinds[js] = 'spacemouse'
        elif (f'ATTRS{{idVendor}}=="{LOGITECH_VENDOR}"' in out and
              f'ATTRS{{idProduct}}=="{LOGITECH_PRODUCT}"' in out and
              any('ATTRS{name}==' in line and LOGITECH_NAME.lower() in line.lower()
                  for line in out.splitlines())):
            kinds[js] = 'logitech'
    return kinds


def _sdl_index_map():
    """Map {/dev/input/jsN: SDL index} for every joystick.

    Jazzy's joy_node passes device_id straight to SDL_JoystickOpen(int), which
    wants the CONTIGUOUS SDL index (0..N-1), NOT the /dev/input/jsN suffix —
    js2 can be SDL index 1, so passing device_id=2 fails and that joystick
    never opens (the right SpaceMouse bug). We query SDL for each joystick's
    event-node path and bridge eventN -> jsN through the by-path symlink pairs
    that share a physical port (usb-0:1.4.1-event-joystick <-> usb-0:1.4.1-joystick).
    """
    sdl_map = {}
    try:
        import ctypes
        import ctypes.util
        sdl = ctypes.CDLL(ctypes.util.find_library('SDL2') or 'libSDL2-2.0.so.0')
        sdl.SDL_Init(0x00000200)  # SDL_INIT_JOYSTICK
        sdl.SDL_NumJoysticks.restype = ctypes.c_int
        sdl.SDL_JoystickPathForIndex.restype = ctypes.c_char_p
        n = sdl.SDL_NumJoysticks()

        # Build port -> event node / js node from the by-path symlinks,
        # preferring the plain usb- variant over the usbv2- one.
        port_to_event, port_to_js = {}, {}
        for link in sorted(glob.glob('/dev/input/by-path/*-joystick')):
            if '-usbv2-' in link:
                continue
            base = os.path.basename(link)
            if base.endswith('-event-joystick'):
                port_to_event[base[: -len('-event-joystick')]] = os.path.realpath(link)
            elif base.endswith('-joystick'):
                port_to_js[base[: -len('-joystick')]] = os.path.realpath(link)

        # SDL index -> event node -> js node
        event_to_js = {e: j for p, e in port_to_event.items()
                       for j in (port_to_js.get(p),) if j}
        for i in range(n):
            event = (sdl.SDL_JoystickPathForIndex(i) or b'').decode()
            js = event_to_js.get(event)
            if js:
                sdl_map[js] = i
        sdl.SDL_Quit()
    except Exception as exc:
        print(f'[spacemouse_unified_teleop] SDL index map unavailable '
              f'({exc}); falling back to contiguous js* order.')
    if not sdl_map:
        # Degraded fallback: assume SDL enumerates in jsN order.
        sdl_map = {js: i for i, js in enumerate(sorted(glob.glob('/dev/input/js*')))}
    return sdl_map


def _sdl_index(path):
    """jsN suffix (str) for a /dev/input/jsN path — NOT the SDL index.

    Kept only as a last-resort fallback when the SDL map is unavailable.
    """
    m = re.search(r'js(\d+)$', path)
    return m.group(1) if m else '0'


def _detect_spacemice(kinds, sdl_map):
    """Return the two SpaceMouse SDL indices as (left, right).

    Picks the classified SpaceMice, then sorts by physical USB path so the
    assignment is deterministic regardless of device-discovery order. Returns
    the CONTIGUOUS SDL index each device maps to — joy_node's device_id is an
    SDL_JoystickOpen index, not the jsN suffix (js2 can be SDL index 1).

    Returns (left_sdl_id, right_sdl_id) as strings, or (None, None) if fewer
    than 2 SpaceMice are found.
    """
    spacemice = sorted((js for js, k in kinds.items() if k == 'spacemouse'),
                       key=lambda p: int(_sdl_index(p)))

    if len(spacemice) < 2:
        print(f'[spacemouse_unified_teleop] WARNING: Expected 2 SpaceMice, '
              f'found {len(spacemice)}. Falling back to device_id 0 and 1.')
        return None, None

    # Resolve physical USB path for deterministic left/right assignment.
    # setdefault on a sorted glob keeps the plain usb- variant over usbv2-
    # (same physical port, either would do — just stay consistent).
    js_to_usbpath = {}
    for symlink in sorted(glob.glob('/dev/input/by-path/*-joystick')):
        if '-event-' in symlink:
            continue
        try:
            real = os.path.realpath(symlink)
            if real in spacemice:
                js_to_usbpath.setdefault(real, symlink)
        except Exception:
            continue

    if len(js_to_usbpath) >= 2:
        # Sort by USB path string for deterministic assignment
        sorted_usb = sorted(js_to_usbpath.items(), key=lambda x: x[1])
        left_js = sorted_usb[0][0]
        right_js = sorted_usb[1][0]
    else:
        # Fall back to js* numeric order
        left_js = spacemice[0]
        right_js = spacemice[1]
        print(f'[spacemouse_unified_teleop] WARNING: Could not resolve USB '
              f'paths; falling back to js* order for left/right assignment.')

    # SDL indices (contiguous) via the SDL map; jsN only as last resort.
    left_id = str(sdl_map.get(left_js, _sdl_index(left_js)))
    right_id = str(sdl_map.get(right_js, _sdl_index(right_js)))

    left_usb = js_to_usbpath.get(left_js, left_js)
    right_usb = js_to_usbpath.get(right_js, right_js)

    print(f'[spacemouse_unified_teleop] Auto-detected SpaceMice:')
    print(f'  LEFT  arm ← SDL {left_id}  ({left_usb})')
    print(f'  RIGHT arm ← SDL {right_id} ({right_usb})')

    return left_id, right_id


def _detect_logitech(kinds, sdl_map):
    """Return the SDL index (str) of the Logitech Extreme 3D, or None."""
    for js, k in kinds.items():
        if k == 'logitech':
            return str(sdl_map.get(js, _sdl_index(js)))
    return None


def launch_setup(context):
    ffw_spacemouse_dir = get_package_share_directory('ffw_spacemouse')
    mapper_launch_file = os.path.join(ffw_spacemouse_dir, 'launch', 'spacemouse_mapper.launch.py')  # file name unchanged (launch file rename would break external refs)

    hardware_mode_str = LaunchConfiguration('hardware_mode').perform(context)
    hardware_mode = hardware_mode_str.lower() in ('true', '1', 'yes')
    robot_model = LaunchConfiguration('robot_model').perform(context)

    # One scan up front classifies every joystick (SpaceMouse vs Logitech) by
    # USB vendor/product, so the two are never confused even when one is
    # unplugged — a missing Logitech must not steal a SpaceMouse's device slot.
    kinds = _scan_joysticks()
    # Map every joystick to its contiguous SDL index — joy_node's device_id
    # is an SDL_JoystickOpen index, not the jsN suffix.
    sdl_map = _sdl_index_map()

    # Resolve device IDs — auto-detect if left at defaults
    left_id = LaunchConfiguration('left_device_id').perform(context)
    right_id = LaunchConfiguration('right_device_id').perform(context)

    # Logitech joystick override. Resolve its real SDL index; if none is
    # connected we skip the logitech node entirely (never launch against
    # device 0, which would be a SpaceMouse).
    use_logitech_str = LaunchConfiguration('use_logitech').perform(context)
    use_logitech = use_logitech_str.lower() in ('true', '1', 'yes')
    logitech_device_id = LaunchConfiguration('logitech_device_id').perform(context)
    logitech_id = None
    if use_logitech:
        if logitech_device_id != '-1':
            logitech_id = logitech_device_id
        else:
            logitech_id = _detect_logitech(kinds, sdl_map)
            if logitech_id is None:
                print('[spacemouse_unified_teleop] WARNING: use_logitech:=true '
                      'but no Logitech Extreme 3D Pro detected — skipping '
                      'logitech teleop rather than grabbing a SpaceMouse.')

    # Quest controller ARM override
    use_quest_str = LaunchConfiguration('use_quest').perform(context)
    use_quest = use_quest_str.lower() in ('true', '1', 'yes')
    quest_device = LaunchConfiguration('quest_device').perform(context)

    if left_id == '0' and right_id == '1':
        auto_left, auto_right = _detect_spacemice(kinds, sdl_map)
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
                'quest_debug_engage': LaunchConfiguration('quest_debug_engage'),
                'quest_debug_error': LaunchConfiguration('quest_debug_error'),
                'quest_thumb_engage_min': LaunchConfiguration('quest_thumb_engage_min'),
                'quest_trigger_engage_min': LaunchConfiguration('quest_trigger_engage_min'),
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
                'quest_debug_engage': LaunchConfiguration('quest_debug_engage'),
                'quest_debug_error': LaunchConfiguration('quest_debug_error'),
                'quest_thumb_engage_min': LaunchConfiguration('quest_thumb_engage_min'),
                'quest_trigger_engage_min': LaunchConfiguration('quest_trigger_engage_min'),
            }.items()
        )
    )

    # Base Teleop Node
    base_teleop_node = Node(
        package='ffw_spacemouse',
        executable='joy_base',
        name='joy_base',
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
    quest_debug_solver_str = LaunchConfiguration('quest_debug_solver').perform(context)
    quest_debug_solver = quest_debug_solver_str.lower() in ('true', '1', 'yes')
    nodes.append(
        Node(
            package='ffw_collision_checker',
            executable='ffw_ik_solver_teleop',
            name='ffw_ik_solver_teleop',
            output='screen',
            parameters=[{
                'hardware_mode': hardware_mode,
                'robot_model': robot_model,
                'quest_debug_solver': quest_debug_solver,
            }]
        )
    )

    # Logitech Joystick (optional — base control override). Only launched when
    # a real Logitech was detected (or an explicit device id was given), so it
    # never opens a SpaceMouse by mistake.
    if use_logitech and logitech_id is not None:
        logitech_launch = os.path.join(
            ffw_spacemouse_dir, 'launch', 'logitech_teleop.launch.py')
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(logitech_launch),
                launch_arguments={
                    'device_id': logitech_id,
                    'max_linear_vel': '0.6',
                    'max_angular_vel': '0.5',
                    'head_step': '0.05',
                    'elevation_goal_a': '-0.1',
                    'elevation_goal_b': '0.15',
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
        DeclareLaunchArgument(
            'quest_debug_engage', default_value='false',
            description='Forward to joy_hand: log each quest engage edge (arm/hold/disarm) for diagnostics.'),
        DeclareLaunchArgument(
            'quest_debug_error', default_value='false',
            description='Forward to joy_hand: print the APPROACH error budget (ep/er + position distances) for slow->fast diagnostics.'),
        DeclareLaunchArgument(
            'quest_thumb_engage_min', default_value='0.5',
            description='Forward to joy_hand: thumb/grip analog must be >= this for the 4-way engage hold.'),
        DeclareLaunchArgument(
            'quest_trigger_engage_min', default_value='0.5',
            description='Forward to joy_hand: trigger analog must be >= this for the 4-way engage hold.'),
        DeclareLaunchArgument(
            'quest_debug_solver', default_value='false',
            description='Forward to ffw_ik_solver_teleop: print the pre-clip target vs achieved pose + solveStep result for quest slow->fast diagnostics.'),
        OpaqueFunction(function=launch_setup),
    ])
