from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Base link name for the kinematic chain'
    )

    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='arm_r_link7',
        description='End effector link name for the kinematic chain'
    )

    num_tests_arg = DeclareLaunchArgument(
        'num_tests',
        default_value='10',
        description='Number of comparison tests to run'
    )

    # IK Solver Comparison Node
    ik_solver_comparison_node = Node(
        package='ffw_kinematics',
        executable='ik_solver_comparison',
        name='ik_solver_comparison',
        output='screen',
        parameters=[
            {'base_link': LaunchConfiguration('base_link')},
            {'end_effector_link': LaunchConfiguration('end_effector_link')},
            {'num_tests': LaunchConfiguration('num_tests')}
        ]
    )

    return LaunchDescription([
        base_link_arg,
        end_effector_link_arg,
        num_tests_arg,
        ik_solver_comparison_node
    ])
