import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import subprocess
import glob

def get_3dconnexion_joysticks():
    joysticks = []
    for js in sorted(glob.glob('/dev/input/js*')):
        try:
            output = subprocess.check_output(f'udevadm info -a -n {js}', shell=True, text=True, stderr=subprocess.DEVNULL)
            if '3Dconnexion' in output:
                joysticks.append(js)
        except Exception:
            pass
    return joysticks

def generate_launch_description():
    ffw_spacemouse_dir = get_package_share_directory('ffw_spacemouse')
    
    # Auto-detect joysticks
    devices = get_3dconnexion_joysticks()
    if len(devices) < 2:
        print(f"WARNING: Expected 2 SpaceMice, but found {len(devices)}. Defaulting to js0 and js1.")
        dev_left = '/dev/input/js0'
        dev_right = '/dev/input/js1'
    else:
        dev_left = devices[0]
        dev_right = devices[1]
        print(f"Auto-assigned LEFT arm to {dev_left}")
        print(f"Auto-assigned RIGHT arm to {dev_right}")

    return LaunchDescription([
        # LEFT ARM MAPPER
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ffw_spacemouse_dir, 'launch', 'spacemouse_mapper.launch.py')
            ),
            launch_arguments={
                'target_arm': 'left', 
                'device_id': dev_left.replace('/dev/input/js', '')
            }.items()
        ),
        
        # RIGHT ARM MAPPER
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ffw_spacemouse_dir, 'launch', 'spacemouse_mapper.launch.py')
            ),
            launch_arguments={
                'target_arm': 'right', 
                'device_id': dev_right.replace('/dev/input/js', '')
            }.items()
        ),
        
        # Launch the MuJoCo QP IK solver teleop node
        Node(
            package='ffw_collision_checker',
            executable='ffw_ik_solver_teleop',
            name='ffw_ik_solver_teleop',
            output='screen',
            parameters=[
                {'rate_hz': 50.0}
            ]
        )
    ])
