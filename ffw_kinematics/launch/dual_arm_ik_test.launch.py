from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for testing the dual arm IK solver and trajectory commander separately.
    This is useful for debugging and development.
    """

    # Launch arguments
    trajectory_duration_arg = DeclareLaunchArgument(
        'trajectory_duration',
        default_value='0.1',
        description='Duration for trajectory execution in seconds'
    )

    enable_gripper_control_arg = DeclareLaunchArgument(
        'enable_gripper_control',
        default_value='true',
        description='Enable gripper control via VR squeeze values'
    )

    # Dual Arm IK Solver node only
    dual_arm_ik_solver_node = Node(
        package='ffw_kinematics',
        executable='dual_arm_ik_solver',
        name='dual_arm_ik_solver',
        output='screen',
        parameters=[{
            'base_link': 'base_link',
            'arm_base_link': 'arm_base_link',
            'right_end_effector_link': 'arm_r_link7',
            'left_end_effector_link': 'arm_l_link7',
            'right_target_pose_topic': '/right__poses',
            'left_target_pose_topic': '/left_poses',
        }]
    )

    # Dual Arm Trajectory Commander node only
    dual_arm_trajectory_commander_node = Node(
        package='ffw_kinematics',
        executable='dual_arm_trajectory_commander',
        name='dual_arm_trajectory_commander',
        output='screen',
        parameters=[{
            'trajectory_duration': LaunchConfiguration('trajectory_duration'),
            'enable_gripper_control': LaunchConfiguration('enable_gripper_control'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        trajectory_duration_arg,
        enable_gripper_control_arg,

        # Nodes
        dual_arm_ik_solver_node,
        dual_arm_trajectory_commander_node,
    ])
