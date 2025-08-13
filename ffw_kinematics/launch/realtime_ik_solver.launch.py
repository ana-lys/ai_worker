from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
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

    target_pose_topic_arg = DeclareLaunchArgument(
        'target_pose_topic',
        default_value='/target_pose',
        description='Topic name for receiving target poses'
    )

    # Realtime IK Solver Node
    realtime_ik_solver_node = Node(
        package='ffw_test',
        executable='realtime_ik_solver',
        name='realtime_ik_solver',
        output='screen',
        parameters=[
            {'base_link': LaunchConfiguration('base_link')},
            {'end_effector_link': LaunchConfiguration('end_effector_link')},
            {'target_pose_topic': LaunchConfiguration('target_pose_topic')}
        ]
    )

    # Target Pose Publisher Node (starts after 5 seconds delay)
    target_pose_publisher_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ffw_kinematics',
                executable='target_pose_publisher',
                name='target_pose_publisher',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        base_link_arg,
        end_effector_link_arg,
        target_pose_topic_arg,
        realtime_ik_solver_node,
        target_pose_publisher_node
    ])
