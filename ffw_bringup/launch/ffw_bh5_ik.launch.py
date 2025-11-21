from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
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

    return LaunchDescription([
        # Frame/link configuration
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('arm_base_link', default_value='arm_base_link'),
        DeclareLaunchArgument('right_end_effector_link', default_value='arm_r_link7'),
        DeclareLaunchArgument('left_end_effector_link', default_value='arm_l_link7'),

        # Topics
        DeclareLaunchArgument('right_target_pose_topic', default_value='/vr_hand/right_wrist'),
        DeclareLaunchArgument('left_target_pose_topic', default_value='/vr_hand/left_wrist'),
        DeclareLaunchArgument('right_ik_solution_topic', default_value='/leader/joint_trajectory_command_broadcaster_right/joint_trajectory'),
        DeclareLaunchArgument('left_ik_solution_topic', default_value='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory'),
        DeclareLaunchArgument('right_current_pose_topic', default_value='/right_current_end_effector_pose'),
        DeclareLaunchArgument('left_current_pose_topic', default_value='/left_current_end_effector_pose'),

        # Offsets for lift joint origin (from URDF)
        DeclareLaunchArgument('lift_joint_x_offset', default_value='0.0055'),
        DeclareLaunchArgument('lift_joint_y_offset', default_value='0.0'),
        DeclareLaunchArgument('lift_joint_z_offset', default_value='1.6316'),

        # IK parameters
        DeclareLaunchArgument('max_joint_step_degrees', default_value='50.0'),
        DeclareLaunchArgument('ik_max_iterations', default_value='100'),
        DeclareLaunchArgument('ik_tolerance', default_value='1e-5'),

        # Hybrid IK parameters
        DeclareLaunchArgument('use_hybrid_ik', default_value='true'),
        DeclareLaunchArgument('current_position_weight', default_value='0.2'),
        DeclareLaunchArgument('previous_solution_weight', default_value='0.8'),

        # Joint limits selection
        DeclareLaunchArgument('use_hardcoded_joint_limits', default_value='true'),

        # Low-pass filter between current state and IK target
        DeclareLaunchArgument('lpf_alpha', default_value='0.9'),

        Node(
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
    ])



