#!/usr/bin/env python3

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
# Author: Wonho Yun

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FfwArmTrajectoryCommander(Node):
    def __init__(self):
        super().__init__('ffw_arm_trajectory_commander')

        # Initialize state variables
        self.left_squeeze_value_ = 0.0
        self.right_squeeze_value_ = 0.0
        self.has_right_ik_solution_ = False
        self.has_left_ik_solution_ = False

        # Declare parameters
        self.declare_parameter('trajectory_duration', 0.01)  # 10ms
        self.declare_parameter('vr_squeeze_closed', 0.035)
        self.declare_parameter('vr_squeeze_open', 0.095)
        self.declare_parameter('gripper_pos_closed', 1.2)
        self.declare_parameter('gripper_pos_open', 0.0)

        # Get parameters
        self.trajectory_duration_ = self.get_parameter('trajectory_duration').value
        self.vr_squeeze_closed_ = self.get_parameter('vr_squeeze_closed').value
        self.vr_squeeze_open_ = self.get_parameter('vr_squeeze_open').value
        self.gripper_pos_closed_ = self.get_parameter('gripper_pos_closed').value
        self.gripper_pos_open_ = self.get_parameter('gripper_pos_open').value

        self.get_logger().info("Dual-arm trajectory commander initialized")

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
        self.right_ik_solution_sub_ = self.create_subscription(
            JointTrajectory,
            '/right_arm_ik_solution',
            self.right_ik_solution_callback,
            qos_profile
        )

        self.left_ik_solution_sub_ = self.create_subscription(
            JointTrajectory,
            '/left_arm_ik_solution',
            self.left_ik_solution_callback,
            qos_profile
        )

        self.left_squeeze_sub_ = self.create_subscription(
            Float32,
            '/vr_hand/left_squeeze',
            self.left_squeeze_callback,
            qos_profile
        )

        self.right_squeeze_sub_ = self.create_subscription(
            Float32,
            '/vr_hand/right_squeeze',
            self.right_squeeze_callback,
            qos_profile
        )

        # Publishers
        self.right_joint_trajectory_pub_ = self.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            qos_profile
        )

        self.left_joint_trajectory_pub_ = self.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            qos_profile
        )

        # Store IK solutions
        self.right_ik_solution_ = JointState()
        self.left_ik_solution_ = JointState()

    def right_ik_solution_callback(self, msg):
        """Callback for right arm IK solution"""
        if not msg.joint_names or not msg.points:
            self.get_logger().warn("Received empty right IK solution")
            return

        point = msg.points[-1]  # Get the latest point

        if len(msg.joint_names) != len(point.positions):
            self.get_logger().error(
                f"Right IK solution: joint names ({len(msg.joint_names)}) and "
                f"positions ({len(point.positions)}) size mismatch"
            )
            return

        # Check for NaN or infinite values
        for pos in point.positions:
            if not np.isfinite(pos):
                self.get_logger().error(f"Right IK solution contains invalid joint position: {pos}")
                return

        self.get_logger().debug(f"Received RIGHT arm IK solution with {len(point.positions)} joints")

        self.right_ik_solution_.name = msg.joint_names
        self.right_ik_solution_.position = point.positions
        self.has_right_ik_solution_ = True

        self.send_right_arm_trajectory()

    def left_ik_solution_callback(self, msg):
        """Callback for left arm IK solution"""
        if not msg.joint_names or not msg.points:
            self.get_logger().warn("Received empty left IK solution")
            return

        point = msg.points[-1]  # Get the latest point

        if len(msg.joint_names) != len(point.positions):
            self.get_logger().error(
                f"Left IK solution: joint names ({len(msg.joint_names)}) and "
                f"positions ({len(point.positions)}) size mismatch"
            )
            return

        # Check for NaN or infinite values
        for pos in point.positions:
            if not np.isfinite(pos):
                self.get_logger().error(f"Left IK solution contains invalid joint position: {pos}")
                return

        self.get_logger().debug(f"Received LEFT arm IK solution with {len(point.positions)} joints")

        self.left_ik_solution_.name = msg.joint_names
        self.left_ik_solution_.position = point.positions
        self.has_left_ik_solution_ = True

        self.send_left_arm_trajectory()

    def left_squeeze_callback(self, msg):
        """Callback for left hand squeeze value"""
        self.left_squeeze_value_ = msg.data

        gripper_position = self.calculate_gripper_position(self.left_squeeze_value_)

        self.get_logger().debug(
            f"Left squeeze: {self.left_squeeze_value_:.3f} → gripper: {gripper_position:.3f}"
        )

        if self.has_left_ik_solution_:
            self.send_left_arm_trajectory()

    def right_squeeze_callback(self, msg):
        """Callback for right hand squeeze value"""
        self.right_squeeze_value_ = msg.data

        gripper_position = self.calculate_gripper_position(self.right_squeeze_value_)

        self.get_logger().debug(
            f"Right squeeze: {self.right_squeeze_value_:.3f} → gripper: {gripper_position:.3f}"
        )

        if self.has_right_ik_solution_:
            self.send_right_arm_trajectory()

    def send_right_arm_trajectory(self):
        """Send right arm trajectory with gripper"""
        if not self.has_right_ik_solution_:
            self.get_logger().warn("No right IK solution available")
            return

        trajectory_msg = JointTrajectory()
        trajectory_msg.header.frame_id = "base_link"

        # Set joint names (arm joints + gripper joints)
        trajectory_msg.joint_names = list(self.right_ik_solution_.name)
        trajectory_msg.joint_names.append("gripper_r_joint1")

        point = JointTrajectoryPoint()
        point.positions = list(self.right_ik_solution_.position)

        gripper_position = self.calculate_gripper_position(self.right_squeeze_value_)
        point.positions.append(gripper_position)

        point.velocities = [0.0] * len(point.positions)
        point.accelerations = [0.0] * len(point.positions)

        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0

        trajectory_msg.points = [point]

        self.right_joint_trajectory_pub_.publish(trajectory_msg)

        self.get_logger().debug(f"Published RIGHT arm trajectory with {len(trajectory_msg.joint_names)} joints")

    def send_left_arm_trajectory(self):
        """Send left arm trajectory with gripper"""
        if not self.has_left_ik_solution_:
            self.get_logger().warn("No left IK solution available")
            return

        trajectory_msg = JointTrajectory()
        trajectory_msg.header.frame_id = "base_link"
        trajectory_msg.joint_names = list(self.left_ik_solution_.name)
        trajectory_msg.joint_names.append("gripper_l_joint1")

        point = JointTrajectoryPoint()
        point.positions = list(self.left_ik_solution_.position)

        gripper_position = self.calculate_gripper_position(self.left_squeeze_value_)
        point.positions.append(gripper_position)

        point.velocities = [0.0] * len(point.positions)
        point.accelerations = [0.0] * len(point.positions)

        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0

        trajectory_msg.points = [point]

        self.left_joint_trajectory_pub_.publish(trajectory_msg)

        self.get_logger().debug(f"Published LEFT arm trajectory with {len(trajectory_msg.joint_names)} joints")

    def calculate_gripper_position(self, squeeze_value):
        """Calculate gripper position from squeeze value"""
        if squeeze_value < self.vr_squeeze_closed_ or squeeze_value > self.vr_squeeze_open_:
            self.get_logger().warn(
                f"VR squeeze value {squeeze_value:.3f} out of expected range "
                f"[{self.vr_squeeze_closed_:.3f}, {self.vr_squeeze_open_:.3f}]"
            )

        # Normalize squeeze value to [0, 1] range
        normalized = (squeeze_value - self.vr_squeeze_closed_) / (self.vr_squeeze_open_ - self.vr_squeeze_closed_)
        normalized = max(0.0, min(1.0, normalized))

        gripper_position = self.gripper_pos_closed_ - (normalized * (self.gripper_pos_closed_ - self.gripper_pos_open_))

        return max(self.gripper_pos_open_, min(self.gripper_pos_closed_, gripper_position))


def main(args=None):
    rclpy.init(args=args)

    node = FfwArmTrajectoryCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
