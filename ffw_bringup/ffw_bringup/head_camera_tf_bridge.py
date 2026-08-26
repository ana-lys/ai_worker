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
# Author: Pyo (modified for head camera TF bridge)
#
# Head camera TF bridge.
#
# The OAK-D head camera frame (`head_camera_frame`) is wired into the robot TF
# tree by a static_transform_publisher that parents it under
# `zedm_left_camera_optical_frame`. That makes `head_camera_frame` -> ...
# -> `arm_base_link` -> `base_link` resolvable through the head joints, but the
# frame then has exactly one TF parent — so this node must NOT broadcast a
# second TF edge. Instead it resolves the transform once per timer tick and
# republishes it on a topic, so the grab planner gets a camera->control-frame
# transform it can consume without touching the TF tree.
#
# The published message is the transform that takes a point expressed in the
# head camera coordinates and expresses it in the control frame, i.e. the
# same thing `lookup_transform(control_frame, head_camera_frame)` returns:
#     p_control = R * p_camera + t
#
# Parameters:
#   control_frame    Frame the end-effector is controlled in (default base_link;
#                    set to 'map' if goals are published in the map frame).
#   camera_frame     Frame detections are expressed in (default head_camera_frame).
#   topic            Topic to republish the resolved transform on.
#   update_rate      Republish rate in Hz.
#   timeout          TF lookup timeout in seconds.
#   enable_debug_logging  Periodically log the resolved transform.

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
from tf2_ros import TransformException


class HeadCameraTfBridge(Node):
    """Publish the head camera -> control-frame transform on a topic."""

    def __init__(self):
        super().__init__('head_camera_tf_bridge')

        # Declare parameters
        self.declare_parameter('control_frame', 'base_link')
        self.declare_parameter('camera_frame', 'head_camera_frame')
        self.declare_parameter('topic', '/head_camera_tf')
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('enable_debug_logging', False)

        # Load parameters
        self.control_frame = self.get_parameter('control_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.topic = self.get_parameter('topic').value
        self.update_rate = self.get_parameter('update_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.enable_debug_logging = self.get_parameter(
            'enable_debug_logging').value

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Transient-local so a late-starting consumer immediately receives the
        # most recent transform.
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.tf_pub = self.create_publisher(
            TransformStamped,
            self.topic,
            qos_profile
        )

        # Throttled warning state for TF lookup failures and debug logging.
        self.fail_counter = 0
        self.debug_counter = 0
        self.warn_interval = 10

        # Periodic republish
        timer_period = 1.0 / self.update_rate
        self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Head Camera TF Bridge initialized')
        self.get_logger().info(f'  Control frame: {self.control_frame}')
        self.get_logger().info(f'  Camera frame:  {self.camera_frame}')
        self.get_logger().info(f'  Publishing to: {self.topic}')
        self.get_logger().info(f'  Update rate:   {self.update_rate} Hz')

    def timer_callback(self):
        """Look up the camera -> control transform and republish it."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.control_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.timeout)
            )
        except TransformException as ex:
            self.fail_counter += 1
            if self.fail_counter % self.warn_interval == 1:
                self.get_logger().warn(
                    f'Could not transform {self.camera_frame} to '
                    f'{self.control_frame}: {ex} '
                    f'(misses so far: {self.fail_counter})'
                )
            return
        self.fail_counter = 0

        # Republish as a topic message (NOT a TF broadcast — head_camera_frame
        # already has a static parent in the TF tree).
        msg = TransformStamped()
        msg.header.stamp = transform.header.stamp
        msg.header.frame_id = self.control_frame
        msg.child_frame_id = self.camera_frame
        msg.transform = transform.transform
        self.tf_pub.publish(msg)

        self.debug_counter += 1
        if self.enable_debug_logging and self.debug_counter % self.warn_interval == 1:
            t = msg.transform.translation
            q = msg.transform.rotation
            # Roll/pitch/yaw from quaternion, for a human-readable readout.
            sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.get_logger().info(
                f'{self.camera_frame} in {self.control_frame}: '
                f'xyz=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) '
                f'rpy=({math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, '
                f'{math.degrees(yaw):.2f}) deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = HeadCameraTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
