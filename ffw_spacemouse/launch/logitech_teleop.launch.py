"""Launch Logitech Extreme 3D Pro full teleop (base + head + EE elevation).

Auto-detects the Logitech, starts the driver and the logitech_base_teleop node.

Usage:
    ros2 launch ffw_spacemouse logitech_teleop.launch.py
    ros2 launch ffw_spacemouse logitech_teleop.launch.py device_id:=1
    ros2 launch ffw_spacemouse logitech_teleop.launch.py max_linear_vel:=1.0 max_angular_vel:=0.8
    ros2 launch ffw_spacemouse logitech_teleop.launch.py head_step:=0.03125 elevation_goal_a:=-0.2
"""

import glob
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions


LOGITECH_VENDOR = '046d'
LOGITECH_PRODUCT = 'c215'
LOGITECH_NAME = 'Extreme 3D'


def _sdl_index_map():
    """Map {/dev/input/jsN: SDL index} so joy_node gets a valid device_id.

    Jazzy's joy_node passes device_id straight to SDL_JoystickOpen(int), which
    wants the CONTIGUOUS SDL index (0..N-1), NOT the /dev/input/jsN suffix
    (js2 can be SDL index 1). Query SDL for each joystick's event-node path
    and bridge eventN -> jsN via the by-path symlink pairs that share a
    physical port (usb-0:1.4.1-event-joystick <-> usb-0:1.4.1-joystick).
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

        port_to_event, port_to_js = {}, {}
        for link in sorted(glob.glob('/dev/input/by-path/*-joystick')):
            if '-usbv2-' in link:      # prefer the plain usb- variant
                continue
            base = os.path.basename(link)
            if base.endswith('-event-joystick'):
                port_to_event[base[: -len('-event-joystick')]] = os.path.realpath(link)
            elif base.endswith('-joystick'):
                port_to_js[base[: -len('-joystick')]] = os.path.realpath(link)

        event_to_js = {e: j for p, e in port_to_event.items()
                       for j in (port_to_js.get(p),) if j}
        for i in range(n):
            event = (sdl.SDL_JoystickPathForIndex(i) or b'').decode()
            js = event_to_js.get(event)
            if js:
                sdl_map[js] = i
        sdl.SDL_Quit()
    except Exception as exc:
        print(f'[logitech_teleop] SDL index map unavailable ({exc}); '
              f'falling back to contiguous js* order.')
    if not sdl_map:
        sdl_map = {js: i for i, js in enumerate(sorted(glob.glob('/dev/input/js*')))}
    return sdl_map


def _detect_logitech():
    """Return the SDL device_id (int) for the Logitech, or None.

    udevadm uses camelCase keys (ATTRS{idVendor}); the old slash form
    (ATTRS{id/vendor}) never matched, so a missing Logitech fell back to
    device 0 and grabbed a SpaceMouse. Also maps jsN -> SDL index: joy_node's
    device_id is the contiguous SDL index, not the jsN suffix. Returns None
    (and refuses to launch) if the Logitech exists but has no SDL index, so
    we never grab a SpaceMouse's slot by guessing.
    """
    sdl_map = _sdl_index_map()
    for js in sorted(glob.glob('/dev/input/js*')):
        try:
            out = subprocess.check_output(
                ['udevadm', 'info', '-a', '-n', js],
                text=True, stderr=subprocess.DEVNULL)
            if (f'ATTRS{{idVendor}}=="{LOGITECH_VENDOR}"' in out and
                f'ATTRS{{idProduct}}=="{LOGITECH_PRODUCT}"' in out and
                any('ATTRS{name}==' in line and LOGITECH_NAME.lower() in line.lower()
                    for line in out.splitlines())):
                if js in sdl_map:
                    return sdl_map[js]
                print(f'[logitech_teleop] WARNING: Logitech found at {js} but '
                      f'it has no SDL index — refusing to guess a device_id.')
                return None
        except Exception:
            continue
    return None


def launch_setup(context):
    device_id_str = LaunchConfiguration('device_id').perform(context)
    device_id = int(device_id_str) if device_id_str != '-1' else _detect_logitech()

    max_lin = float(LaunchConfiguration('max_linear_vel').perform(context))
    max_ang = float(LaunchConfiguration('max_angular_vel').perform(context))
    head_step = float(LaunchConfiguration('head_step').perform(context))
    elevation_goal_a = float(LaunchConfiguration('elevation_goal_a').perform(context))
    elevation_goal_b = float(LaunchConfiguration('elevation_goal_b').perform(context))

    if device_id is None:
        print('[logitech_teleop] ERROR: Logitech Extreme 3D Pro not detected — '
              'NOT launching (would otherwise grab device 0, a SpaceMouse). '
              'Plug it in or pass device_id:=<n>.', flush=True)
        return []

    print(f'[logitech_teleop] Logitech device_id={device_id}', flush=True)

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
                'head_step': head_step,
                'elevation_goal_a': elevation_goal_a,
                'elevation_goal_b': elevation_goal_b,
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
        DeclareLaunchArgument(
            'head_step', default_value='0.05',
            description='Head position increment per hat press (rad).'),
        DeclareLaunchArgument(
            'elevation_goal_a', default_value='-0.1',
            description='EE Z offset goal A (meters, button Top-4).'),
        DeclareLaunchArgument(
            'elevation_goal_b', default_value='0.15',
            description='EE Z offset goal B (meters, button Top-5).'),
        OpaqueFunction(function=launch_setup),
    ])
