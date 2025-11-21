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

from geometry_msgs.msg import Point, PoseStamped, Quaternion
import nest_asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32
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
        self.vr_publishing_enabled = False  # Default: disabled

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

        self.fps = 100
        self.get_logger().info(f'VR Trajectory server available at: https://{hostname}:8012')

        # VR event handlers
        self.vuer.add_handler('HAND_MOVE')(self.on_hand_move)
        self.vuer.add_handler('CAMERA_MOVE')(self.on_camera_move)

        # Publishers
        self.head_joint_pub = self.create_publisher(
            JointTrajectory,
            '/leader/joystick_controller_left/joint_trajectory',
            10
        )

        self.left_squeeze_pub = self.create_publisher(Float32, '/vr_hand/left_squeeze', 10)
        self.right_squeeze_pub = self.create_publisher(Float32, '/vr_hand/right_squeeze', 10)

        # Wrist pose publishers for visualization
        self.left_wrist_rviz_pub = self.create_publisher(PoseStamped, '/vr_hand/left_wrist', 10)
        self.right_wrist_rviz_pub = self.create_publisher(PoseStamped, '/vr_hand/right_wrist', 10)

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

        # Head pitch offset configuration
        self.pitch_offset = -0.5  # Adjustable pitch offset in radians

        # Logging counters
        self.hand_log_counter = 0
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
                self.get_logger().info('VR publishing disabled')

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

    def get_joint_matrix(self, hand_data, joint_index):
        """Extract joint transformation matrix from hand data."""
        arr = np.array(hand_data)
        start_idx = joint_index * 16
        end_idx = start_idx + 16
        matrix_data = arr[start_idx:end_idx]
        return matrix_data.reshape(4, 4, order='F')

    def publish_wrist_pose(self, hand_data, side='left'):
        """Publish wrist pose for visualization."""
        try:
            # Use wrist joint (index 0) for wrist pose
            world_joint_matrix = self.get_joint_matrix(hand_data, 0)
            relative_joint_matrix = self.head_inverse_matrix @ world_joint_matrix
            relative_pos, relative_quat = self.matrix_to_pose(relative_joint_matrix)
            relative_pos_ros, relative_quat_ros = self.vr_to_ros_transform(relative_pos, relative_quat)

            # Fixed offset: zedm_camera_center → base_link
            zedm_to_base_offset = np.array([
                0.0 - 0.0238122 - 0.040 - 0.049483 - 0.0055,  # x: -0.1187952
                0.0 + 0.0 + 0.0 + 0.0 + 0.0,                  # y: 0.0
                -0.01325 + 0.0242094 - 0.054 - 0.102130 - 1.4316  # z: -1.5767706
            ], dtype=np.float64)

            # Transform from camera relative coordinates to base_link coordinates
            base_position = relative_pos_ros - zedm_to_base_offset

            # Use camera relative orientation as is
            camera_relative_rotation = R.from_quat(relative_quat_ros)

            # Additional: Apply 180-degree rotation around Z-axis (right hand only)
            if side == 'right':
                rot_z_180 = R.from_euler('z', 180, degrees=True)
                camera_relative_rotation = camera_relative_rotation * rot_z_180

            arm_quaternion = camera_relative_rotation.as_quat()  # [x, y, z, w]

            # Create wrist pose message
            wrist_pose = PoseStamped()
            wrist_pose.header.stamp = self.get_clock().now().to_msg()
            wrist_pose.header.frame_id = 'base_link'

            wrist_pose.pose.position = self.safe_point(
                base_position[0], base_position[1], base_position[2]
            )
            wrist_pose.pose.orientation = self.safe_quaternion(
                arm_quaternion[0], arm_quaternion[1],
                arm_quaternion[2], arm_quaternion[3]
            )

            # Publish to appropriate topic
            if side == 'left':
                self.left_wrist_rviz_pub.publish(wrist_pose)
            elif side == 'right':
                self.right_wrist_rviz_pub.publish(wrist_pose)

        except Exception as e:
            self.get_logger().warn(f'Error publishing wrist pose for {side}: {e}')

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

            left_state = event.value.get('leftState', {})
            right_state = event.value.get('rightState', {})

            if left_state and 'squeezeValue' in left_state:
                squeeze_val = left_state['squeezeValue']
                if self.is_valid_float(squeeze_val):
                    left_squeeze_msg = Float32()
                    left_squeeze_msg.data = float(squeeze_val)
                    self.left_squeeze_pub.publish(left_squeeze_msg)

            if right_state and 'squeezeValue' in right_state:
                squeeze_val = right_state['squeezeValue']
                if self.is_valid_float(squeeze_val):
                    right_squeeze_msg = Float32()
                    right_squeeze_msg.data = float(squeeze_val)
                    self.right_squeeze_pub.publish(right_squeeze_msg)

            # Process hand data
            if 'left' in event.value:
                left_data = event.value['left']
                if isinstance(left_data, (list, np.ndarray)) and len(left_data) == 400:
                    self.left_hand_data = np.array(left_data)
                    # Publish left wrist pose
                    self.publish_wrist_pose(self.left_hand_data, 'left')

            # Process hand data
            if 'right' in event.value:
                right_data = event.value['right']
                if isinstance(right_data, (list, np.ndarray)) and len(right_data) == 400:
                    self.right_hand_data = np.array(right_data)
                    # Publish right wrist pose
                    self.publish_wrist_pose(self.right_hand_data, 'right')

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
