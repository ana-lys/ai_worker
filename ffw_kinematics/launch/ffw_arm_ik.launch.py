#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Authors: Wonho Yun

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    trajectory_duration_arg = DeclareLaunchArgument(
        'trajectory_duration',
        default_value='0.01',
        description='Duration for trajectory execution in seconds'
    )

    max_joint_step_degrees_arg = DeclareLaunchArgument(
        'max_joint_step_degrees',
        default_value='20.0',
        description='Maximum joint movement per IK cycle in degrees'
    )

    use_hardcoded_joint_limits_arg = DeclareLaunchArgument(
        'use_hardcoded_joint_limits',
        default_value='true',
        description='Use hardcoded joint limits instead of URDF limits'
    )

    # ffw Arm IK Solver node only
    ffw_arm_ik_solver_node = Node(
        package='ffw_kinematics',
        executable='ffw_arm_ik_solver',
        name='ffw_arm_ik_solver',
        output='screen',
        parameters=[{
            'base_link': 'base_link',
            'arm_base_link': 'arm_base_link',
            'right_end_effector_link': 'arm_r_link7',
            'left_end_effector_link': 'arm_l_link7',
            'right_target_pose_topic': '/vr_hand/right_wrist',
            'left_target_pose_topic': '/vr_hand/left_wrist',
            'max_joint_step_degrees': LaunchConfiguration('max_joint_step_degrees'),
            'use_hardcoded_joint_limits': LaunchConfiguration('use_hardcoded_joint_limits'),
        }]
    )

    # ffw Arm Trajectory Commander node only
    ffw_arm_trajectory_commander_node = Node(
        package='ffw_kinematics',
        executable='ffw_arm_trajectory_commander',
        name='ffw_arm_trajectory_commander',
        output='screen',
        parameters=[{
            'trajectory_duration': LaunchConfiguration('trajectory_duration'),
        }]
    )

    return LaunchDescription([
        trajectory_duration_arg,
        max_joint_step_degrees_arg,
        use_hardcoded_joint_limits_arg,
        ffw_arm_ik_solver_node,
        ffw_arm_trajectory_commander_node,
    ])
