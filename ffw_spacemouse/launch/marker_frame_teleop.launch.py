"""SpaceMouse teleop with the EE goals anchored on the AprilTag board.

Same as spacemouse_unified_teleop.launch.py (all its arguments apply) with
teleop_frame:=marker_frame: joy_hand_marker x2 send goals in marker_frame to
teleop_core, which keeps them fixed on the board while the base moves, gates
on marker health (starts halted until the board is seen) and clamps the base
(MARKER_TELEOP_PLAN.md D3-D7). Needs the marker EKF (ffw_odom) running.

    ros2 launch ffw_spacemouse marker_frame_teleop.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    unified = os.path.join(get_package_share_directory('ffw_spacemouse'),
                           'launch', 'spacemouse_unified_teleop.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unified),
            launch_arguments={
                'teleop_frame': 'marker_frame',
                'use_teleop_core': 'true',
            }.items()),
    ])
