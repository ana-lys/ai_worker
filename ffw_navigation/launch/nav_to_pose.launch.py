# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Minimal Nav2 launch for swerve pose-to-pose navigation.

No map server, no AMCL, no RViz. Just Nav2 (planner + MPPI Omni controller
+ costmaps + BT navigator) using EKF odom.

Usage:
    ros2 launch ffw_navigation nav_to_pose.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_navigation = get_package_share_directory('ffw_navigation')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            pkg_navigation,
            'config',
            'navigation_mppi.yaml'
        ),
        description='Full path to the Nav2 params file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        navigation_launch,
    ])
