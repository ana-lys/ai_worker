import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ffw_spacemouse_dir = get_package_share_directory('ffw_spacemouse')
    mapper_launch_file = os.path.join(ffw_spacemouse_dir, 'launch', 'spacemouse_mapper.launch.py')

    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='true',
        description='If true, enables base teleop, mode switching, and hardware/Gazebo sync.'
    )

    left_device_id_arg = DeclareLaunchArgument('left_device_id', default_value='0')
    right_device_id_arg = DeclareLaunchArgument('right_device_id', default_value='1')

    # Left SpaceMouse Mapper (includes joy_node)
    left_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapper_launch_file),
        launch_arguments={
            'target_arm': 'left',
            'device_id': LaunchConfiguration('left_device_id')
        }.items()
    )

    # Right SpaceMouse Mapper (includes joy_node)
    right_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapper_launch_file),
        launch_arguments={
            'target_arm': 'right',
            'device_id': LaunchConfiguration('right_device_id')
        }.items()
    )

    # Base Teleop Node
    base_teleop_node = Node(
        condition=IfCondition(LaunchConfiguration('hardware_mode')),
        package='ffw_spacemouse',
        executable='spacemouse_base_teleop',
        name='spacemouse_base_teleop',
        output='screen',
        parameters=[{
            'base_joy_topic': '/right/joy',
            'aux_joy_topic':  '/left/joy',
            'head_topic': '/leader/joystick_controller_left/joint_trajectory',
            'max_linear_vel': 0.3,
            'max_angular_vel': 0.5,
            'head_step': 0.03125,
            'axis_x': 1,
            'axis_y': 0,
            'axis_yaw': 5,
            'axis_z': 2,
            'axis_pitch': 4,
            'axis_head_pan': 3,
            'invert_x': False,
            'invert_y': False,
            'invert_yaw': False,
            'invert_z': False,
            'invert_pitch': False,
            'invert_head_pan': False,
        }]
    )

    # IK Solver Teleop Node
    ik_solver_node = Node(
        package='ffw_collision_checker',
        executable='ffw_ik_solver_teleop',
        name='ffw_ik_solver_teleop',
        output='screen',
        parameters=[{
            'hardware_mode': LaunchConfiguration('hardware_mode')
        }]
    )

    # Failsafe shutdown event
    failsafe_event = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration('hardware_mode')),
        event_handler=OnProcessExit(
            target_action=base_teleop_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='SpaceMouse Base Teleop terminated due to Joint State Failsafe!'))
            ]
        )
    )

    return LaunchDescription([
        hardware_mode_arg,
        left_device_id_arg,
        right_device_id_arg,
        left_mapper,
        right_mapper,
        base_teleop_node,
        ik_solver_node,
        failsafe_event
    ])
