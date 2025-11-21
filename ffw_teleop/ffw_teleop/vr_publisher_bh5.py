#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Wonho Yun

import asyncio
import math
import os
import socket
import threading

from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
import nest_asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vuer import Vuer
from vuer.schemas import Hands
from std_msgs.msg import Bool

# Allow nested asyncio execution
nest_asyncio.apply()


class VRTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('vr_trajectory_publisher')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # VR publishing control flag
        self.vr_publishing_enabled = True #False  # Default: disabled

        # VR Server setup
        current_dir = os.path.dirname(os.path.abspath(__file__))
        cert_file = os.path.join(current_dir, 'cert.pem')
        key_file = os.path.join(current_dir, 'key.pem')
        hostname = socket.gethostbyname(socket.gethostname())
        ws_url = f'ws://{hostname}:8012'

        self.vuer = Vuer(
            host='0.0.0.0',
            port=8012,
            cert=cert_file,
            key=key_file,
            ws=ws_url,
            queries=dict(grid=False, reconnect=True),
            queue_len=3
        )

        self.fps = 30
        self.get_logger().info(f'VR Trajectory server available at: https://{hostname}:8012')

        # VR event handlers
        self.vuer.add_handler('HAND_MOVE')(self.on_hand_move)
        self.vuer.add_handler('CAMERA_MOVE')(self.on_camera_move)

        # Hand joint configuration
        self.left_joint_names = [
            "finger_l_joint1", "finger_l_joint2", "finger_l_joint3", "finger_l_joint4",
            "finger_l_joint5", "finger_l_joint6", "finger_l_joint7", "finger_l_joint8",
            "finger_l_joint9", "finger_l_joint10", "finger_l_joint11", "finger_l_joint12",
            "finger_l_joint13", "finger_l_joint14", "finger_l_joint15", "finger_l_joint16",
            "finger_l_joint17", "finger_l_joint18", "finger_l_joint19", "finger_l_joint20"
        ]

        self.right_joint_names = [
            "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
            "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
            "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
            "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
            "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"
        ]

        self.left_joint_positions = [0.0] * 20
        self.right_joint_positions = [0.0] * 20

        self.min_joint_limits = [
            -2.2, -2.0, 0.0, 0.0,
            -0.6, 0.0, 0.0, 0.0,
            -0.6, 0.0, 0.0, 0.0,
            -0.6, 0.0, 0.0, 0.0,
            -0.6, 0.0, 0.0, 0.0
        ]

        self.max_joint_limits = [
            0.0, 0.3, 1.57, 1.57,
            0.6, 2.0, 1.57, 1.57,
            0.6, 2.0, 1.57, 1.57,
            0.6, 2.0, 1.57, 1.57,
            0.6, 2.0, 1.57, 1.57,
        ]

        # Publishers - Direct trajectory publishing without PoseArray
        self.left_hand_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/left_hand/joint_trajectory',
            10
        )
        self.right_hand_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/right_hand/joint_trajectory',
            10
        )
        self.head_joint_pub = self.create_publisher(
            JointTrajectory,
            '/leader/joystick_controller_left/joint_trajectory',
            10
        )
        self.left_wrist_rviz_pub = self.create_publisher(PoseStamped, '/vr_hand/left_wrist', 10)
        self.right_wrist_rviz_pub = self.create_publisher(PoseStamped, '/vr_hand/right_wrist', 10)

        self.left_thumb_pub = self.create_publisher(PoseArray, '/vr_hand/left_thumb', 10)
        self.right_thumb_pub = self.create_publisher(PoseArray, '/vr_hand/right_thumb', 10)

        # Subscriber for VR control toggle
        self.vr_control_sub = self.create_subscription(
            Bool,
            '/vr_control/toggle',
            self.vr_control_callback,
            10
        )

        # VR data storage
        self.left_hand_data = None
        self.right_hand_data = None
        self.head_transform_matrix = np.eye(4)
        self.head_inverse_matrix = np.eye(4)

        # Low-pass filter settings
        self.low_pass_filter_alpha = 0.3

        # Scaling VR data
        self.scaling_vr = 1.1

        # Head pitch offset configuration
        self.pitch_offset = -0.5  # Adjustable pitch offset in radians

        self.hand_log_counter = 0

        # Timer for hand trajectory publishing
        # self.timer_period = 0.005  # 200 Hz for smooth trajectory generation
        self.timer_period = 0.01  # 100 Hz for smooth trajectory generation
        self.timer = self.create_timer(self.timer_period, self.publish_hand_trajectory)

        # Status monitoring timer (every 5 seconds)
        self.status_timer = self.create_timer(5.0, self.log_status)

        # Logging counters
        self.head_log_counter = 0
        self.log_every_n = self.fps

        # Async setup
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.start_vuer_server()

        self.get_logger().info('VR Trajectory Publisher node has been started')
        self.get_logger().info('VR publishing is DISABLED by default. Send /vr_control/toggle message (True=enable, False=disable).')

    def vr_control_callback(self, msg):
        """Callback to enable/disable VR publishing based on message content."""
        new_state = bool(msg.data)  # Read message content

        # Only log if state actually changed
        if new_state != self.vr_publishing_enabled:
            self.vr_publishing_enabled = new_state
            status = "ENABLED" if self.vr_publishing_enabled else "DISABLED"
            self.get_logger().info(f'VR publishing changed to: {status} (message value: {msg.data})')

            if not self.vr_publishing_enabled:
                # Reset joint positions to zero when disabled
                self.left_joint_positions = [0.0] * 20
                self.right_joint_positions = [0.0] * 20
                self.get_logger().info('Joint positions reset to zero')

    def log_status(self):
        """Log current system status for debugging."""
        vr_status = "ENABLED" if self.vr_publishing_enabled else "DISABLED"

        self.get_logger().info(f'Status: VR={vr_status}')

        # Check if we have hand data
        left_data_status = "Available" if self.left_hand_data is not None else "None"
        right_data_status = "Available" if self.right_hand_data is not None else "None"

        self.get_logger().info(f'Hand data: Left={left_data_status}, Right={right_data_status}')

        # Check joint positions
        if self.left_joint_positions and any(pos != 0.0 for pos in self.left_joint_positions):
            self.get_logger().info(f'Left joint positions: {self.left_joint_positions[:5]}...')
        if self.right_joint_positions and any(pos != 0.0 for pos in self.right_joint_positions):
            self.get_logger().info(f'Right joint positions: {self.right_joint_positions[:5]}...')


    def is_valid_float(self, value):
        """Check if value is valid float (excluding NaN, inf)."""
        return isinstance(value, (int, float)) and np.isfinite(value)

    def safe_point(self, x, y, z):
        """Create safe Point (filtering NaN/inf values)."""
        safe_x = float(x) if self.is_valid_float(x) else 0.0
        safe_y = float(y) if self.is_valid_float(y) else 0.0
        safe_z = float(z) if self.is_valid_float(z) else 0.0
        return Point(x=safe_x, y=safe_y, z=safe_z)

    def safe_quaternion(self, x, y, z, w):
        """Create safe Quaternion (filtering NaN/inf values)."""
        safe_x = float(x) if self.is_valid_float(x) else 0.0
        safe_y = float(y) if self.is_valid_float(y) else 0.0
        safe_z = float(z) if self.is_valid_float(z) else 0.0
        safe_w = float(w) if self.is_valid_float(w) else 1.0
        return Quaternion(x=safe_x, y=safe_y, z=safe_z, w=safe_w)

    def matrix_to_pose(self, mat):
        """Convert 4x4 transformation matrix to (position, quaternion)."""
        pos = mat[:3, 3]
        rot = mat[:3, :3]

        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(rot)):
            self.get_logger().warn('Invalid matrix data detected, using default pose')
            return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])

        trace = rot[0, 0] + rot[1, 1] + rot[2, 2]

        if 1 + trace <= 0:
            quat = np.array([0.0, 0.0, 0.0, 1.0])
            return pos, quat

        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (rot[2, 1] - rot[1, 2]) / s
            qy = (rot[0, 2] - rot[2, 0]) / s
            qz = (rot[1, 0] - rot[0, 1]) / s
        elif ((rot[0, 0] > rot[1, 1]) and (rot[0, 0] > rot[2, 2])):
            s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif rot[1, 1] > rot[2, 2]:
            s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s

        quat = np.array([qx, qy, qz, qw])

        if not np.all(np.isfinite(quat)):
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            norm = np.linalg.norm(quat)
            if norm > 0:
                quat = quat / norm
            else:
                quat = np.array([0.0, 0.0, 0.0, 1.0])

        return pos, quat

    def vr_to_ros_transform(self, vr_pos, vr_quat):
        """Transform from VR coordinate system to ROS coordinate system."""
        vr_to_ros_matrix = np.array([
            [0, 0, -1],
            [-1, 0, 0],
            [0, 1, 0]
        ])

        ros_pos = vr_to_ros_matrix @ vr_pos

        vr_rotation = R.from_quat([vr_quat[0], vr_quat[1], vr_quat[2], vr_quat[3]])
        vr_rot_matrix = vr_rotation.as_matrix()
        ros_rot_matrix = vr_to_ros_matrix @ vr_rot_matrix
        ros_rotation = R.from_matrix(ros_rot_matrix)
        ros_quat = ros_rotation.as_quat()

        return ros_pos, ros_quat

    def transform_and_publish_pose(self, pose_array_msg, publisher, hand_name, vr_scale=1.0):
        """Transform pose from head relative coordinates to base_link and publish."""
        if not pose_array_msg.poses:
            return

        # Assume the first pose in the array is the wrist pose (head relative, ROS coordinates)
        wrist_pose_relative = pose_array_msg.poses[0]

        # Extract relative position (head/camera relative, already in ROS coordinate system)
        camera_relative_position = np.array([
            wrist_pose_relative.position.x * vr_scale,
            wrist_pose_relative.position.y * vr_scale,
            wrist_pose_relative.position.z * vr_scale
        ], dtype=np.float64)

        # Extract relative orientation (head/camera relative, already in ROS coordinate system)
        camera_relative_quaternion = np.array([
            wrist_pose_relative.orientation.x,
            wrist_pose_relative.orientation.y,
            wrist_pose_relative.orientation.z,
            wrist_pose_relative.orientation.w
        ], dtype=np.float64)

        # Fixed offset: zedm_camera_center → base_link
        zedm_to_base_offset = np.array([
            0.0 - 0.0238122 - 0.040 - 0.049483 - 0.0055,  # x: -0.1187952
            0.0 + 0.0 + 0.0 + 0.0 + 0.0,                  # y: 0.0
            -0.01325 + 0.0242094 - 0.054 - 0.102130 - 1.4316  # z: -1.5767706
        ], dtype=np.float64)

        # Transform from camera relative coordinates directly to base_link coordinates
        base_position = camera_relative_position - zedm_to_base_offset

        # Use camera relative orientation as is
        camera_relative_rotation = R.from_quat(camera_relative_quaternion)

        # Additional: Apply 180-degree rotation around Z-axis (right hand only)
        if hand_name == 'right':
            rot_z_180 = R.from_euler('z', 180, degrees=True)
            camera_relative_rotation = camera_relative_rotation * rot_z_180

        arm_quaternion = camera_relative_rotation.as_quat()  # [x, y, z, w]

        # Create target pose message
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'base_link'

        target_pose.pose.position.x = base_position[0]-0.15  # Small offset for better visualization
        target_pose.pose.position.y = base_position[1]
        target_pose.pose.position.z = base_position[2]

        target_pose.pose.orientation.x = arm_quaternion[0]
        target_pose.pose.orientation.y = arm_quaternion[1]
        target_pose.pose.orientation.z = arm_quaternion[2]
        target_pose.pose.orientation.w = arm_quaternion[3]

        publisher.publish(target_pose)

        self.get_logger().debug(
            'Transformed {} pose: pos=[{:.3f}, {:.3f}, {:.3f}]'.format(
                hand_name, base_position[0], base_position[1], base_position[2]
            )
        )

    def get_joint_matrix(self, hand_data, joint_index):
        """Extract joint transformation matrix from hand data."""
        arr = np.array(hand_data)
        start_idx = joint_index * 16
        end_idx = start_idx + 16
        matrix_data = arr[start_idx:end_idx]
        return matrix_data.reshape(4, 4, order='F')

    def quat_inverse(self, q):
        """Returns the inverse of a quaternion."""
        norm = q.x**2 + q.y**2 + q.z**2 + q.w**2
        if norm == 0:
            return Quaternion()
        inv_norm = 1.0 / norm
        msg = Quaternion()
        msg.x = -q.x * inv_norm
        msg.y = -q.y * inv_norm
        msg.z = -q.z * inv_norm
        msg.w = q.w * inv_norm
        return msg

    def quat_multiply(self, q1, q2):
        """Multiply two quaternions."""
        w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        msg = Quaternion()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.w = w
        return msg

    def get_roll_pitch_yaw(self, q1, q2, cmd=''):
        """Calculate roll, pitch, yaw from two quaternions."""
        q_combined = self.quat_multiply(q1, q2)
        w, x, y, z = q_combined.w, q_combined.x, q_combined.y, q_combined.z

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1, 1)
        pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        if cmd == 'r':
            return roll
        elif cmd == 'p':
            return pitch
        elif cmd == 'y':
            return yaw
        else:
            return roll, pitch, yaw

    def wrap_pi(self, angle):
        """Wrap angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def process_hand_joints(self, hand_data, side='left'):
        """Process VR hand data and calculate joint positions with low-pass filtering."""
        if hand_data is None or len(hand_data) != 400:
            return

        # Extract hand pose quaternions
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = ''
        pose_array.poses = []
        poses = []
        for i in range(25):
            try:
                world_joint_matrix = self.get_joint_matrix(hand_data, i)
                relative_joint_matrix = self.head_inverse_matrix @ world_joint_matrix
                relative_pos, relative_quat = self.matrix_to_pose(relative_joint_matrix)
                relative_pos_ros, relative_quat_ros = self.vr_to_ros_transform(relative_pos, relative_quat)

                quat = Quaternion()
                quat.x = relative_quat_ros[0]
                quat.y = relative_quat_ros[1]
                quat.z = relative_quat_ros[2]
                quat.w = relative_quat_ros[3]
                poses.append(quat)

                if (i == 0) or ((i >= 2) and (i <= 4)):
                    pose_msg = Pose()
                    pose_msg.position = self.safe_point(relative_pos_ros[0], relative_pos_ros[1], relative_pos_ros[2])
                    pose_msg.orientation = quat
                    pose_array.poses.append(pose_msg)


            except Exception as e:
                self.get_logger().warn(f'Error processing hand joint {i}: {e}')
                return

        if len(poses) >= 24:  # Need at least 24 poses for all joints
            # Calculate joint angles using the same logic as hand_joint_state_publisher
            quat0 = poses[0]   # wrist
            quat6 = poses[6]
            quat7 = poses[7]
            quat8 = poses[8]
            quat11 = poses[11]
            quat12 = poses[12]
            quat13 = poses[13]
            quat16 = poses[16]
            quat17 = poses[17]
            quat18 = poses[18]
            quat21 = poses[21]
            quat22 = poses[22]
            quat23 = poses[23]

            # Create temporary joint positions
            temp_joint_positions = [0.0] * 20

            # Index finger (joints 4-7)
            temp_joint_positions[4] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat6, 'p') * self.scaling_vr)
            temp_joint_positions[5] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat6, 'r') * self.scaling_vr)
            temp_joint_positions[6] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat6), quat7, 'r') * self.scaling_vr)
            temp_joint_positions[7] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat7), quat8, 'r') * self.scaling_vr)

            # Middle finger (joints 8-11)
            temp_joint_positions[8] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat11, 'p') * self.scaling_vr)
            temp_joint_positions[9] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat11, 'r') * self.scaling_vr)
            temp_joint_positions[10] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat11), quat12, 'r') * self.scaling_vr)
            temp_joint_positions[11] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat12), quat13, 'r') * self.scaling_vr)

            # Ring finger (joints 12-15)
            temp_joint_positions[12] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat16, 'p') * self.scaling_vr)
            temp_joint_positions[13] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat16, 'r') * self.scaling_vr)
            temp_joint_positions[14] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat16), quat17, 'r') * self.scaling_vr)
            temp_joint_positions[15] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat17), quat18, 'r') * self.scaling_vr)

            # Pinky finger (joints 16-19)
            temp_joint_positions[16] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat21, 'p') * self.scaling_vr)
            temp_joint_positions[17] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat21, 'r') * self.scaling_vr)
            temp_joint_positions[18] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat21), quat22, 'r') * self.scaling_vr)
            temp_joint_positions[19] = self.wrap_pi(-self.get_roll_pitch_yaw(self.quat_inverse(quat22), quat23, 'r') * self.scaling_vr)

            if side == 'left':
                if pose_array.poses:
                    self.left_thumb_pub.publish(pose_array)

                    # Transform and publish left wrist pose to base_link coordinates
                    self.transform_and_publish_pose(
                        pose_array, self.left_wrist_rviz_pub, 'left', vr_scale=1.7
                    )

                # Apply low-pass filter
                # TODO: change to np.array to compute in one line
                for i in range(20):
                    self.left_joint_positions[i] = (temp_joint_positions[i] * self.low_pass_filter_alpha +
                                            (1 - self.low_pass_filter_alpha) * self.left_joint_positions[i])
            elif side == 'right':
                if pose_array.poses:
                    self.right_thumb_pub.publish(pose_array)

                    # Transform and publish right wrist pose to base_link coordinates
                    self.transform_and_publish_pose(
                        pose_array, self.right_wrist_rviz_pub, 'right', vr_scale=1.7
                    )

                # Apply low-pass filter
                for i in range(20):
                    self.right_joint_positions[i] = (temp_joint_positions[i] * self.low_pass_filter_alpha +
                                            (1 - self.low_pass_filter_alpha) * self.right_joint_positions[i])

    def publish_hand_trajectory(self):
        """Publish hand joint trajectory directly only if enabled."""
        if not self.vr_publishing_enabled:
            return

        left_msg = JointTrajectory()
        left_msg.joint_names = self.left_joint_names
        left_goal_point = JointTrajectoryPoint()
        left_goal_point.positions = self.left_joint_positions.copy()
        left_goal_point.time_from_start.sec = 0
        left_goal_point.time_from_start.nanosec = 0
        left_msg.points.append(left_goal_point)
        self.left_hand_trajectory_pub.publish(left_msg)

        right_msg = JointTrajectory()
        right_msg.joint_names = self.right_joint_names
        right_goal_point = JointTrajectoryPoint()
        right_goal_point.positions = self.right_joint_positions.copy()
        right_goal_point.time_from_start.sec = 0
        right_goal_point.time_from_start.nanosec = 0
        right_msg.points.append(right_goal_point)
        self.right_hand_trajectory_pub.publish(right_msg)

    def start_vuer_server(self):
        """Start the VR server in a separate thread."""
        async def start_server():
            try:
                self.vuer.spawn(start=True)(self.main_hand_tracking)
                self.get_logger().info('Starting VR server...')
                await self.vuer.start()
            except Exception as e:
                self.get_logger().error(f'Failed to start VR server: {e}')

        def run_server():
            try:
                self.loop.run_until_complete(start_server())
                self.loop.run_forever()
            except Exception as e:
                self.get_logger().error(f'Error in VR server thread: {e}')

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    async def main_hand_tracking(self, session):
        """Main hand tracking session."""
        try:
            fps = self.fps
            self.get_logger().info('Starting hand tracking session')
            session.upsert(
                Hands(
                    fps=fps,
                    stream=True,
                    key='hands',
                    hideLeft=False,
                    hideRight=False,
                ),
                to='bgChildren',
            )
            self.get_logger().info('Hand tracking enabled')
            while True:
                await asyncio.sleep(1/fps)
        except Exception as e:
            self.get_logger().error(f'Error in hand tracking session: {e}')

    async def on_camera_move(self, event, session):
        """Handle camera movement events."""
        try:
            if not self.vr_publishing_enabled:
                return

            if getattr(event, 'key', None) != 'defaultCamera':
                return

            matrix = None
            if isinstance(event.value, dict):
                if 'matrix' in event.value:
                    matrix = event.value['matrix']
                elif 'camera' in event.value and isinstance(event.value['camera'], dict):
                    matrix = event.value['camera'].get('matrix')

            if isinstance(matrix, (list, np.ndarray)) and len(matrix) == 16:
                self.head_transform_matrix = np.array(matrix).reshape(4, 4, order='F')
                self.head_inverse_matrix = np.linalg.inv(self.head_transform_matrix)
                pos, quat = self.matrix_to_pose(self.head_transform_matrix)

                # Publish head joint trajectory
                r = R.from_quat(quat)
                roll, pitch, yaw = r.as_euler('zxy')

                if self.is_valid_float(pitch) and self.is_valid_float(yaw):
                    msg = JointTrajectory()
                    msg.joint_names = ['head_joint1', 'head_joint2']
                    point = JointTrajectoryPoint()
                    # Apply pitch offset
                    adjusted_pitch = pitch + self.pitch_offset
                    point.positions = [-float(adjusted_pitch), float(yaw)]
                    point.velocities = [0.0, 0.0]
                    point.accelerations = [0.0, 0.0]
                    point.effort = []
                    msg.points.append(point)
                    self.head_joint_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error in camera move event: {e}')

    async def on_hand_move(self, event, session):
        """Handle hand movement events."""
        try:
            if not self.vr_publishing_enabled:
                return

            # Process hand data
            if 'left' in event.value:
                left_data = event.value['left']
                if isinstance(left_data, (list, np.ndarray)) and len(left_data) == 400:
                    self.left_hand_data = np.array(left_data)
                    # Process left hand joints directly
                    self.process_hand_joints(self.left_hand_data,'left')

            # Process hand data
            if 'right' in event.value:
                right_data = event.value['right']
                if isinstance(right_data, (list, np.ndarray)) and len(right_data) == 400:
                    self.right_hand_data = np.array(right_data)
                    # Process right hand joints directly
                    self.process_hand_joints(self.right_hand_data,'right')

            # Debug logging
            self.hand_log_counter += 1
            if self.hand_log_counter % self.log_every_n == 0:
                self.get_logger().info('Hand tracking data processed')

        except Exception as e:
            self.get_logger().error(f'Error in hand move event: {e}')

    def __del__(self):
        try:
            if hasattr(self, 'vuer'):
                self.loop.run_until_complete(self.vuer.stop())
            if hasattr(self, 'loop'):
                self.loop.close()
        except Exception as e:
            self.get_logger().error(f'Error in cleanup: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VRTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
