import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_joystick_id():
    """Find the best available RadioLink joystick device ID."""
    try:
        result = subprocess.check_output(
            ['ros2', 'run', 'joy', 'joy_enumerate_devices'],
            text=True,
            stderr=subprocess.DEVNULL,
        )

        # Priority 1: R16F
        if "RadioLink R16F" in result:
            for line in result.splitlines():
                if "RadioLink R16F" in line:
                    return int(line.split(':')[0].strip())

        # Priority 2: T16D
        if "Radiolink T16D" in result:
            for line in result.splitlines():
                if "Radiolink T16D" in line:
                    return int(line.split(':')[0].strip())

    except Exception:
        pass

    return 0


def get_joint_limits_from_urdf():
    """Load required joint limits from ffw_description URDF."""
    # Fallback defaults (used only if URDF read fails)
    defaults = {
        'left_arm_lower_limits': [-3.14, -0.1, -3.14, -2.9361, -3.14, -1.57, -1.8201, 0.0],
        'left_arm_upper_limits': [3.14, 3.14, 3.14, 1.0786, 3.14, 1.57, 1.5804, 1.1],
        'right_arm_lower_limits': [-3.14, -3.14, -3.14, -2.9361, -3.14, -1.57, -1.5804, 0.0],
        'right_arm_upper_limits': [3.14, 0.1, 3.14, 1.0786, 3.14, 1.57, 1.8201, 1.1],
        'head_lower_limits': [-0.2317, -0.35],
        'head_upper_limits': [0.6951, 0.35],
        'lift_lower_limits': [-0.5],
        'lift_upper_limits': [0.0],
    }

    try:
        desc_share = Path(get_package_share_directory('ffw_description'))
        urdf_path = desc_share / 'urdf' / 'ffw_sg2_rev1_follower' / 'ffw_sg2_follower.urdf'

        root = ET.parse(urdf_path).getroot()
        limits = {}
        for joint in root.findall('joint'):
            name = joint.attrib.get('name')
            limit = joint.find('limit')
            if limit is not None and ('lower' in limit.attrib and 'upper' in limit.attrib):
                limits[name] = (float(limit.attrib['lower']), float(limit.attrib['upper']))

        left_joints = [
            'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
            'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'gripper_l_joint1'
        ]
        right_joints = [
            'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
            'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'gripper_r_joint1'
        ]
        head_joints = ['head_joint1', 'head_joint2']
        lift_joints = ['lift_joint']

        def build_limits(joint_names):
            lower = []
            upper = []
            for j in joint_names:
                lo, hi = limits[j]
                lower.append(lo)
                upper.append(hi)
            return lower, upper

        left_lower, left_upper = build_limits(left_joints)
        right_lower, right_upper = build_limits(right_joints)
        head_lower, head_upper = build_limits(head_joints)
        lift_lower, lift_upper = build_limits(lift_joints)

        return {
            'left_arm_lower_limits': left_lower,
            'left_arm_upper_limits': left_upper,
            'right_arm_lower_limits': right_lower,
            'right_arm_upper_limits': right_upper,
            'head_lower_limits': head_lower,
            'head_upper_limits': head_upper,
            'lift_lower_limits': lift_lower,
            'lift_upper_limits': lift_upper,
        }
    except Exception:
        return defaults


def generate_launch_description():
    target_id = get_joystick_id()
    joint_limits = get_joint_limits_from_urdf()

    # Same center offset calibration used in radio_display.launch.py
    radio_bias = -0.0381

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': target_id,
                'deadzone': 0.0,
                'autorepeat_rate': 50.0,
            }]
        ),
        Node(
            package='ffw_radio',
            executable='radio_control_mapper',
            name='radio_control_mapper',
            output='screen',
            parameters=[{
                'center_offset': radio_bias,
                'deadband': 0.05,
                'max_linear_speed': 1.0,
                'max_angular_speed': 1.5,
                'max_joint_speed': 0.8,
                'pose_twitch_gain': 1.5,
                'pose_twitch_ticks': 15,
                **joint_limits,
            }]
        ),
    ])
