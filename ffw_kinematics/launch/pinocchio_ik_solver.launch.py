from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('ffw_description'),
        'urdf', 'ffw_bg2_rev4_follower', 'ffw_bg2_follower.urdf'
    ])

    # Read URDF content
    with open(os.path.join(
        get_package_share_directory('ffw_description'),
        'urdf', 'ffw_bg2_rev4_follower', 'ffw_bg2_follower.urdf'
    ), 'r') as urdf_file_handle:
        robot_description = urdf_file_handle.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_robot_description',
            default_value='true',
            description='Use robot_description parameter instead of URDF file'
        ),

        Node(
            package='ffw_kinematics',
            executable='pinocchio_ik_solver',
            name='pinocchio_ik_solver',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_robot_description': LaunchConfiguration('use_robot_description')},
                {'end_effector_link': 'arm_r_link7'},
                {'max_iterations': 1000},
                {'tolerance': 0.3},
                {'step_size': 0.01}
            ]
        )
    ])
