from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, EmitEvent,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false',
        description='Simulation mode: skip joint_states subscription and snap-back'
    )

    # Joy mapper (wraps both joy_node and joy_mapper for arm EE control)
    # Launch two instances for left and right arm EE control.
    right_mapper = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ffw_joy'),
            'launch',
            'joy_mapper.launch.py',
        ]),
        launch_arguments={
            'target_arm': 'right',
            'device_id': '0',
        }.items(),
    )
    left_mapper = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ffw_joy'),
            'launch',
            'joy_mapper.launch.py',
        ]),
        launch_arguments={
            'target_arm': 'left',
            'device_id': '1',
        }.items(),
    )

    # Joy base teleop device joy_nodes (device detection only — no base_teleop Node)
    base_teleop_devices = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ffw_joy'),
            'launch',
            'joy_base_teleop.launch.py',
        ]),
        launch_arguments={
            'sim': LaunchConfiguration('sim'),
            'enable_node': 'false',
        }.items(),
    )

    # Base teleop node — created directly so OnProcessExit can track it
    # (IncludeLaunchDescription wrapping an OpaqueFunction-based launch is not
    #  an ExecuteLocal and can't be used as target_action.)
    base_teleop_node = Node(
        package='ffw_joy',
        executable='joy_base_teleop',
        name='joy_base_teleop',
        output='screen',
        parameters=[{
            'sim': LaunchConfiguration('sim'),
            'right_namespace': 'right',
            'left_namespace': 'left',
            'right_topic': '/right/joy',
            'left_topic': '/left/joy',
            'base_frame': 'base_link',
            'elevator_frame': 'elevator',
            'head_frame': 'head',
            'max_base_lin_vel': 0.5,
            'max_base_ang_vel': 2.0,
            'max_head_vel': 0.5,
            'max_elevator_vel': 0.3,
            'base_sensitivity': 1.0,
            'head_sensitivity': 1.0,
            'elevator_sensitivity': 1.0,
            'logitech_axis_x': 0,
            'logitech_axis_y': 1,
            'logitech_axis_yaw': 3,
            'logitech_axis_throttle': 2,
            'logitech_axis_hat_x': 4,
            'logitech_axis_hat_y': 5,
            'logitech_hat_speed': 0.5,
            'logitech_throttle_power': 1.0,
            'precision_factor': 0.3,
        }]
    )

    # Failsafe shutdown: when base teleop exits unexpectedly (e.g. joint state
    # failsafe), kill the entire launch tree so the robot stops moving.
    failsafe_shutdown = RegisterEventHandler(
        condition=IfCondition(PythonExpression([
            "'false' if '", LaunchConfiguration('sim'), "' == 'true' else 'true'"
        ])),
        event_handler=OnProcessExit(
            target_action=base_teleop_node,
            on_exit=[EmitEvent(event=Shutdown(
                reason='Base teleop terminated — shutting down all components.'
            ))],
        ),
    )

    # IK solver teleop (inverse of sim → hardware_mode)
    ik_solver_node = Node(
        package='ffw_collision_checker',
        executable='ffw_ik_solver_teleop',
        name='ffw_ik_solver_teleop',
        output='screen',
        parameters=[{
            'hardware_mode': PythonExpression([
                "'false' if '", LaunchConfiguration('sim'), "' == 'true' else 'true'"
            ]),
            'robot_model': 'bg2',
        }]
    )

    # Start RViz with pre-configured display
    rviz = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ffw_joy'),
            'launch',
            'visualize.launch.py',
        ]),
        launch_arguments={}.items(),
    )

    return LaunchDescription([
        sim_arg,
        right_mapper,
        left_mapper,
        base_teleop_devices,
        base_teleop_node,
        failsafe_shutdown,
        ik_solver_node,
        rviz,
    ])
