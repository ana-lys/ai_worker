from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for testing the ffw arm IK solver and trajectory commander separately.
    This is useful for debugging and development.
    """

    # Launch arguments
    trajectory_duration_arg = DeclareLaunchArgument(
        'trajectory_duration',
        default_value='0.01',
        description='Duration for trajectory execution in seconds'
    )

    enable_gripper_control_arg = DeclareLaunchArgument(
        'enable_gripper_control',
        default_value='true',
        description='Enable gripper control via VR squeeze values'
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
            'enable_gripper_control': LaunchConfiguration('enable_gripper_control'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        trajectory_duration_arg,
        enable_gripper_control_arg,
        max_joint_step_degrees_arg,
        use_hardcoded_joint_limits_arg,

        # Nodes
        ffw_arm_ik_solver_node,
        ffw_arm_trajectory_commander_node,
    ])
