#!/usr/bin/env python3

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
# Author: Wonho Yun

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    base_link = LaunchConfiguration('base_link')
    arm_base_link = LaunchConfiguration('arm_base_link')
    right_end_effector_link = LaunchConfiguration('right_end_effector_link')
    left_end_effector_link = LaunchConfiguration('left_end_effector_link')

    right_target_pose_topic = LaunchConfiguration('right_target_pose_topic')
    left_target_pose_topic = LaunchConfiguration('left_target_pose_topic')
    right_ik_solution_topic = LaunchConfiguration('right_ik_solution_topic')
    left_ik_solution_topic = LaunchConfiguration('left_ik_solution_topic')
    right_current_pose_topic = LaunchConfiguration('right_current_pose_topic')
    left_current_pose_topic = LaunchConfiguration('left_current_pose_topic')

    lift_joint_x_offset = LaunchConfiguration('lift_joint_x_offset')
    lift_joint_y_offset = LaunchConfiguration('lift_joint_y_offset')
    lift_joint_z_offset = LaunchConfiguration('lift_joint_z_offset')

    max_joint_step_degrees = LaunchConfiguration('max_joint_step_degrees')
    ik_max_iterations = LaunchConfiguration('ik_max_iterations')
    ik_tolerance = LaunchConfiguration('ik_tolerance')

    use_hybrid_ik = LaunchConfiguration('use_hybrid_ik')
    current_position_weight = LaunchConfiguration('current_position_weight')
    previous_solution_weight = LaunchConfiguration('previous_solution_weight')
    use_hardcoded_joint_limits = LaunchConfiguration('use_hardcoded_joint_limits')

    lpf_alpha = LaunchConfiguration('lpf_alpha')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Frame/link configuration
    ld.add_action(DeclareLaunchArgument('base_link', default_value='base_link'))
    ld.add_action(DeclareLaunchArgument('arm_base_link', default_value='arm_base_link'))
    ld.add_action(DeclareLaunchArgument('right_end_effector_link', default_value='arm_r_link7'))
    ld.add_action(DeclareLaunchArgument('left_end_effector_link', default_value='arm_l_link7'))

    # Topics
    ld.add_action(DeclareLaunchArgument('right_target_pose_topic', default_value='/vr_hand/right_wrist'))
    ld.add_action(DeclareLaunchArgument('left_target_pose_topic', default_value='/vr_hand/left_wrist'))
    ld.add_action(DeclareLaunchArgument('right_ik_solution_topic', default_value='/leader/joint_trajectory_command_broadcaster_right/joint_trajectory'))
    ld.add_action(DeclareLaunchArgument('left_ik_solution_topic', default_value='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory'))
    ld.add_action(DeclareLaunchArgument('right_current_pose_topic', default_value='/right_current_end_effector_pose'))
    ld.add_action(DeclareLaunchArgument('left_current_pose_topic', default_value='/left_current_end_effector_pose'))

    # Offsets for lift joint origin (from URDF)
    ld.add_action(DeclareLaunchArgument('lift_joint_x_offset', default_value='0.0055'))
    ld.add_action(DeclareLaunchArgument('lift_joint_y_offset', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('lift_joint_z_offset', default_value='1.6316'))

    # IK parameters
    ld.add_action(DeclareLaunchArgument('max_joint_step_degrees', default_value='50.0'))
    ld.add_action(DeclareLaunchArgument('ik_max_iterations', default_value='100'))
    ld.add_action(DeclareLaunchArgument('ik_tolerance', default_value='1e-5'))

    # Hybrid IK parameters
    ld.add_action(DeclareLaunchArgument('use_hybrid_ik', default_value='true'))
    ld.add_action(DeclareLaunchArgument('current_position_weight', default_value='0.2'))
    ld.add_action(DeclareLaunchArgument('previous_solution_weight', default_value='0.8'))

    # Joint limits selection
    ld.add_action(DeclareLaunchArgument('use_hardcoded_joint_limits', default_value='true'))

    # Low-pass filter between current state and IK target
    ld.add_action(DeclareLaunchArgument('lpf_alpha', default_value='0.9'))

    # Arm IK Solver Node
    arm_ik_solver_node = Node(
        package='ffw_kinematics',
        executable='arm_ik_solver',
        name='arm_ik_solver',
        output='screen',
        parameters=[{
            'base_link': base_link,
            'arm_base_link': arm_base_link,
            'right_end_effector_link': right_end_effector_link,
            'left_end_effector_link': left_end_effector_link,

            'right_target_pose_topic': right_target_pose_topic,
            'left_target_pose_topic': left_target_pose_topic,
            'right_ik_solution_topic': right_ik_solution_topic,
            'left_ik_solution_topic': left_ik_solution_topic,
            'right_current_pose_topic': right_current_pose_topic,
            'left_current_pose_topic': left_current_pose_topic,

            'lift_joint_x_offset': lift_joint_x_offset,
            'lift_joint_y_offset': lift_joint_y_offset,
            'lift_joint_z_offset': lift_joint_z_offset,

            'max_joint_step_degrees': max_joint_step_degrees,
            'ik_max_iterations': ik_max_iterations,
            'ik_tolerance': ik_tolerance,

            'use_hybrid_ik': use_hybrid_ik,
            'current_position_weight': current_position_weight,
            'previous_solution_weight': previous_solution_weight,

            'use_hardcoded_joint_limits': use_hardcoded_joint_limits,

            'lpf_alpha': lpf_alpha,
        }]
    )

    # Hand IK Solver Node
    hand_ik_solver_node = Node(
        package='ffw_kinematics',
        executable='hand_ik_solver',
        name='hand_ik_solver',
        output='screen'
    )

    # Hand Controller Node
    preset_hand_controller_node = Node(
        package='ffw_teleop',
        executable='preset_hand_controller',
        name='preset_hand_controller',
        output='screen'
    )

    # Add the nodes
    ld.add_action(arm_ik_solver_node)
    ld.add_action(hand_ik_solver_node)
    ld.add_action(preset_hand_controller_node)

    return ld
