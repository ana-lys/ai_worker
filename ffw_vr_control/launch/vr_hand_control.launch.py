from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare("ffw_description"),
        "urdf",
        "ffw_bg2_rev3_follower",
        "ffw_bg2_follower.urdf.xacro"
    ])

    return LaunchDescription([
        Node(
            package="ffw_vr_control",
            executable="vr_hand_control_node",
            parameters=[
                {"robot_description": Command(["xacro ", urdf_path])}
            ],
            output="screen"
        )
    ])
