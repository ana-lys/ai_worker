import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from ament_index_python.packages import get_package_share_directory

def get_joint_limits_from_urdf():
    defaults = {
        'arm_lower_limits': [-3.14, -3.14, -3.14, -2.9361, -3.14, -1.57, -1.5804, 0.0],
        'arm_upper_limits': [3.14, 0.1, 3.14, 1.0786, 3.14, 1.57, 1.8201, 1.1],
    }
    return defaults # Simplified for now

def generate_launch_description():
    joint_limits = get_joint_limits_from_urdf()
    
    target_arm_arg = DeclareLaunchArgument('target_arm', default_value='right', description='Target arm: left or right')
    device_id_arg = DeclareLaunchArgument('device_id', default_value='0', description='Device ID integer (e.g., 0 for /dev/input/js0)')

    # Quest override params (quest_teleop_plan §7) — forwarded to joy_hand.
    # Defaults mirror joy_hand.cpp so leaving an arg at its default is a no-op;
    # the unified launch sets quest_publish_notifications=false on the right-hand
    # instance (only the left one fires /ffw_control/notification).
    quest_publish_notifications_arg = DeclareLaunchArgument(
        'quest_publish_notifications', default_value='true',
        description='Publish /ffw_control/notification on APPROACH->READY. Set false on the right-hand instance.')
    quest_hold_engage_s_arg = DeclareLaunchArgument(
        'quest_hold_engage_s', default_value='2.0',
        description='Both-hands thumb+trigger hold time to engage the quest override.')
    quest_release_min_arg = DeclareLaunchArgument(
        'quest_release_min', default_value='0.1',
        description='All four analog floats below this arms the engage count.')
    quest_w_slow_arg = DeclareLaunchArgument(
        'quest_w_slow', default_value='0.15',
        description='Interpolation gain during QUEST_APPROACH.')
    quest_approach_pos_m_arg = DeclareLaunchArgument(
        'quest_approach_pos_m', default_value='0.05',
        description='Position error threshold for APPROACH->READY.')
    quest_approach_ang_rad_arg = DeclareLaunchArgument(
        'quest_approach_ang_rad', default_value='0.10',
        description='Rotation error threshold for APPROACH->READY.')
    quest_timeout_s_arg = DeclareLaunchArgument(
        'quest_timeout_s', default_value='0.5',
        description='No /quest_state for this long -> abort freeze.')
    quest_debug_engage_arg = DeclareLaunchArgument(
        'quest_debug_engage', default_value='false',
        description='Log each detect_engage edge (arm/hold/disarm) in joy_hand for diagnostics.')
    quest_thumb_engage_min_arg = DeclareLaunchArgument(
        'quest_thumb_engage_min', default_value='0.5',
        description='Thumb/grip analog must be >= this for the 4-way engage hold.')
    quest_trigger_engage_min_arg = DeclareLaunchArgument(
        'quest_trigger_engage_min', default_value='0.5',
        description='Trigger analog must be >= this for the 4-way engage hold.')
    quest_debug_error_arg = DeclareLaunchArgument(
        'quest_debug_error', default_value='false',
        description='Print the APPROACH error budget (ep/er + position distances) in joy_hand for slow->fast diagnostics.')

    return LaunchDescription([
        target_arm_arg,
        device_id_arg,
        quest_publish_notifications_arg,
        quest_hold_engage_s_arg,
        quest_release_min_arg,
        quest_w_slow_arg,
        quest_approach_pos_m_arg,
        quest_approach_ang_rad_arg,
        quest_timeout_s_arg,
        quest_debug_engage_arg,
        quest_thumb_engage_min_arg,
        quest_trigger_engage_min_arg,
        quest_debug_error_arg,
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=LaunchConfiguration('target_arm'),
            parameters=[{
                'device_id': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('device_id'), value_type=int),
                'deadzone': 0.0,
                'autorepeat_rate': 100.0,
            }]
        ),
        Node(
            package='ffw_spacemouse',
            executable='joy_hand',
            name='joy_hand',
            namespace=LaunchConfiguration('target_arm'),
            output='screen',
            parameters=[{
                'target_arm': LaunchConfiguration('target_arm'),
                'pos_step': 0.005,
                'rot_step': 0.025,
                'publish_rate_hz': 100.0,
                'trans_sensitivity': 1.5,
                'rot_sensitivity': 1.0,
                'reference_frame': 'global',
                'axis_x': 1,
                'axis_y': 0,
                'axis_z': 2,
                'axis_roll': 4,
                'axis_pitch': 3,
                'axis_yaw': 5,
                'invert_x': False,
                'invert_y': False,
                'invert_z': False,
                'invert_roll': False,
                'invert_pitch': False,
                'invert_yaw': False,
                'ee_orientation_slack_deg': 1.0,
                'quest_publish_notifications': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_publish_notifications'), value_type=bool),
                'quest_hold_engage_s': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_hold_engage_s'), value_type=float),
                'quest_release_min': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_release_min'), value_type=float),
                'quest_w_slow': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_w_slow'), value_type=float),
                'quest_approach_pos_m': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_approach_pos_m'), value_type=float),
                'quest_approach_ang_rad': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_approach_ang_rad'), value_type=float),
                'quest_timeout_s': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_timeout_s'), value_type=float),
                'quest_debug_engage': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_debug_engage'), value_type=bool),
                'quest_debug_error': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_debug_error'), value_type=bool),
                'quest_thumb_engage_min': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_thumb_engage_min'), value_type=float),
                'quest_trigger_engage_min': launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration('quest_trigger_engage_min'), value_type=float),
                **joint_limits,
            }]
        ),
    ])
