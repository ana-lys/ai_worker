"""Launch a joy_node for the Logitech Extreme 3D Pro + the explorer script.

Auto-detects the Logitech by vendor/product/name, starts the driver, and
runs the live input explorer so you can verify every axis and button.

Usage:
    ros2 launch ffw_spacemouse explore_logitech.launch.py
"""

import glob
import os
import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


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
        print(f'[explore_logitech] SDL index map unavailable ({exc}); '
              f'falling back to contiguous js* order.')
    if not sdl_map:
        sdl_map = {js: i for i, js in enumerate(sorted(glob.glob('/dev/input/js*')))}
    return sdl_map


def _detect_logitech():
    """Return the SDL device_id (int) for the Logitech, or None.

    udevadm uses camelCase keys (ATTRS{idVendor}); the old slash form
    (ATTRS{id/vendor}) never matched, so detection always failed and the
    explorer grabbed device 0 (a SpaceMouse) instead. Also maps jsN -> SDL
    index — joy_node's device_id is the contiguous SDL index, not the jsN
    suffix. Returns None if the Logitech exists but has no SDL index.
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
                print(f'[explore_logitech] WARNING: Logitech found at {js} but '
                      f'it has no SDL index — refusing to guess a device_id.')
                return None
        except Exception:
            continue
    return None


def launch_setup(context):
    logitech_id = _detect_logitech()

    if logitech_id is not None:
        print(f'[explore_logitech] Logitech Extreme 3D Pro detected as '
              f'device_id={logitech_id}')
    else:
        # No Logitech recognized — fall back to the first SDL joystick so the
        # explorer is still usable, but say exactly which device that is.
        logitech_id = 0
        sdl_map = _sdl_index_map()
        listed = ', '.join(f'SDL {i} = {js}'
                           for js, i in sorted(sdl_map.items(), key=lambda x: x[1]))
        print(f'[explore_logitech] WARNING: Logitech not found — exploring '
              f'SDL index 0 instead. Available: {listed}')

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
