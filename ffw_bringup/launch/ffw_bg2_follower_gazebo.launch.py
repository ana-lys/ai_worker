#!/usr/bin/env python3

import os
from pathlib import Path
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ffw_description_path = os.path.join(
        get_package_share_directory('ffw_description'))

    ffw_bringup_path = os.path.join(
        get_package_share_directory('ffw_bringup'))

    # Gazebo sim path 설정
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(ffw_bringup_path, 'worlds'), ':' +
            str(Path(ffw_description_path).parent.resolve())
        ]
    )

    # Launch argument 선언
    declare_args = [
        DeclareLaunchArgument(
            'world',
            default_value='empty_world',
            description='Gazebo world to load'
        )
    ]

    # Gazebo launch 포함
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch/gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), '.sdf', ' -v 1', ' -r']
        }.items()
    )

    # Xacro 파싱
    xacro_file = os.path.join(
        ffw_description_path,
        'urdf',
        'ffw_bg2_rev3_follower',
        'ffw_bg2_follower.urdf.xacro'
    )
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    # robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Gazebo에 로봇 스폰
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            '-name', 'ffw',
            '-allow_renaming', 'true',
            '-use_sim', 'true'
        ]
    )

    # 컨트롤러들 로드
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_l_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_l_controller'],
        output='screen'
    )

    load_arm_r_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_r_controller'],
        output='screen'
    )

    load_head_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'head_controller'],
        output='screen'
    )

    load_lift_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'lift_controller'],
        output='screen'
    )

    # clock 브릿지
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # RViz
    rviz_config_file = os.path.join(ffw_description_path, 'rviz', 'ffw.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        *declare_args,
        gazebo_resource_path,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[
                    load_arm_l_controller,
                    load_arm_r_controller,
                    load_head_controller,
                    load_lift_controller
                ]
            )
        ),
        bridge,
        rviz
    ])
