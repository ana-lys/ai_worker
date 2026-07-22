import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import subprocess
import glob

def get_left_right_sdl_ids():
    import glob
    import os
    import subprocess

    # 1. Find all 3Dconnexion joysticks and determine their SDL IDs
    # SDL enumerates joysticks in the order of their /dev/input/js* indices
    js_nodes = []
    for js in sorted(glob.glob('/dev/input/js*')):
        try:
            output = subprocess.check_output(f'udevadm info -a -n {js}', shell=True, text=True, stderr=subprocess.DEVNULL)
            if '3Dconnexion' in output:
                js_nodes.append(js)
        except Exception:
            pass

    sdl_map = {js_nodes[i]: str(i) for i in range(len(js_nodes))}

    # 2. Get their physical USB paths
    path_map = {}
    for symlink in glob.glob('/dev/input/by-path/*-joystick'):
        if '-event-' in symlink:
            continue
        try:
            real_js = os.path.realpath(symlink)
            if real_js in js_nodes:
                path_map[symlink] = real_js
        except Exception:
            pass

    # Sort by physical USB path to consistently assign Left and Right
    sorted_paths = sorted(path_map.keys())

    if len(sorted_paths) < 2:
        print(f"WARNING: Expected 2 SpaceMice, but found {len(sorted_paths)}. Defaulting to SDL IDs 0 and 1.")
        return '0', '1'

    left_js = path_map[sorted_paths[0]]
    right_js = path_map[sorted_paths[1]]

    print(f"Auto-assigned LEFT arm to {sorted_paths[0]} ({left_js}, SDL ID: {sdl_map[left_js]})")
    print(f"Auto-assigned RIGHT arm to {sorted_paths[1]} ({right_js}, SDL ID: {sdl_map[right_js]})")

    return sdl_map[left_js], sdl_map[right_js]

def generate_launch_description():
    ffw_spacemouse_dir = get_package_share_directory('ffw_spacemouse')
    
    sdl_left, sdl_right = get_left_right_sdl_ids()

    sim_only_arg = DeclareLaunchArgument(
        'sim_only', default_value='false',
        description='Run in simulation-only mode (no hardware /joint_states)'
    )

    from launch.substitutions import PythonExpression

    return LaunchDescription([
        sim_only_arg,
        # LEFT ARM MAPPER
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ffw_spacemouse_dir, 'launch', 'spacemouse_mapper.launch.py')
            ),
            launch_arguments={
                'target_arm': 'left', 
                'device_id': sdl_left
            }.items()
        ),
        
        # RIGHT ARM MAPPER
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ffw_spacemouse_dir, 'launch', 'spacemouse_mapper.launch.py')
            ),
            launch_arguments={
                'target_arm': 'right', 
                'device_id': sdl_right
            }.items()
        ),
        
        # Launch the MuJoCo QP IK solver teleop node
        Node(
            package='ffw_collision_checker',
            executable='ffw_ik_solver_teleop',
            name='ffw_ik_solver_teleop',
            output='screen',
            parameters=[
                {'rate_hz': 50.0},
                {'hardware_mode': PythonExpression(["'", LaunchConfiguration('sim_only'), "' != 'true'"])},
                {'robot_model': 'bg2'}
            ]
        )
    ])
