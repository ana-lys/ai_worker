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
#
# IMU tilt correction (head play), opt-in via imu_calib_file:
#   The head tilt joint (head_joint1) has ~1 deg of play the encoder does not
#   report (measured 2026-10-12), so the joint-predicted camera pose can be off
#   by that much in tilt. When the D435i gyro says the camera has been still for
#   imu_still_s, gravity from the accelerometer fixes that one angle: the
#   rotation about head_joint1's axis (head_link1 y, through its pivot) that
#   makes the joint-predicted camera "up" agree with the measured one. It is
#   low-passed (imu_tilt_tau_s), held while the camera moves, and applied to the
#   published transform only -- the TF tree itself stays joint-based.
#   imu_calib_file: head_cam_calib_capture.py --solve output (*_solve.json) with
#   imu.imu_cam_rotvec (IMU->camera rotation correction) and imu.floor_up_rotvec
#   (floor tilt, used as base_link's tilt). '' disables the correction.
#   Status (JSON, 2 Hz): <topic>_imu_tilt.

import collections
import json
import math
import os
import time

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException


def _rotvec_to_R(rv):
    rv = np.asarray(rv, float)
    a = np.linalg.norm(rv)
    if a < 1e-12:
        return np.eye(3)
    k = rv / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


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
        self.declare_parameter('imu_calib_file', '')
        self.declare_parameter('imu_ns', '/d435')
        self.declare_parameter('pivot_frame', 'head_link1')
        self.declare_parameter('imu_still_s', 0.5)
        self.declare_parameter('imu_still_dps', 0.5)
        self.declare_parameter('imu_accel_tol', 0.03)
        self.declare_parameter('imu_tilt_tau_s', 0.5)
        self.declare_parameter('imu_tilt_max_deg', 3.0)

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

        self._init_imu_tilt()

        # Periodic republish
        timer_period = 1.0 / self.update_rate
        self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Head Camera TF Bridge initialized')
        self.get_logger().info(f'  Control frame: {self.control_frame}')
        self.get_logger().info(f'  Camera frame:  {self.camera_frame}')
        self.get_logger().info(f'  Publishing to: {self.topic}')
        self.get_logger().info(f'  Update rate:   {self.update_rate} Hz')

    # ── IMU tilt correction (head play) ─────────────────────────────────
    def _init_imu_tilt(self):
        self.imu_on = False
        self.tilt = 0.0              # applied correction about head_joint1's axis [rad]
        self.tilt_raw = None
        self.tilt_t = None           # wall time of the last static update
        self.tilt_offaxis = None
        self.static = False
        self.last_status = 0.0
        path = os.path.expanduser(self.get_parameter('imu_calib_file').value or '')
        if not path:
            self.get_logger().info('IMU tilt correction: off (imu_calib_file empty)')
            return
        try:
            with open(path) as fh:
                imu = json.load(fh)['imu']
            self.R_ic = _rotvec_to_R(imu['imu_cam_rotvec'])
            self.up_base = _rotvec_to_R(imu['floor_up_rotvec']) @ np.array([0.0, 0.0, 1.0])
        except (OSError, KeyError, TypeError, ValueError) as e:
            self.get_logger().error(f'IMU tilt correction: off, cannot read {path}: {e}')
            return
        self.pivot_frame = self.get_parameter('pivot_frame').value
        self.still_s = self.get_parameter('imu_still_s').value
        self.still_rad = math.radians(self.get_parameter('imu_still_dps').value)
        self.acc_tol = self.get_parameter('imu_accel_tol').value
        self.tau = self.get_parameter('imu_tilt_tau_s').value
        self.tilt_max = math.radians(self.get_parameter('imu_tilt_max_deg').value)
        self.acc = collections.deque(maxlen=1000)
        self.gyr = collections.deque(maxlen=1000)
        ns = self.get_parameter('imu_ns').value
        self.create_subscription(Imu, ns + '/accel/sample', self._on_acc, qos_profile_sensor_data)
        self.create_subscription(Imu, ns + '/gyro/sample', self._on_gyr, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, self.topic + '_imu_tilt', 10)
        self.imu_on = True
        self.get_logger().info(
            f'IMU tilt correction: ON from {path} (IMU->camera '
            f'{np.degrees(imu["imu_cam_rotvec"]).round(3).tolist()} deg, floor '
            f'{np.degrees(imu["floor_up_rotvec"][:2]).round(3).tolist()} deg)')

    def _on_acc(self, m):
        v = m.linear_acceleration
        self.acc.append((time.time(), v.x, v.y, v.z))

    def _on_gyr(self, m):
        v = m.angular_velocity
        self.gyr.append((time.time(), v.x, v.y, v.z))

    def _imu_tilt(self, R_bc):
        """Update the tilt correction when the camera is static; returns
        (pivot point, axis) in the control frame for applying it, or None."""
        try:
            tp = self.tf_buffer.lookup_transform(self.control_frame, self.pivot_frame, rclpy.time.Time())
        except TransformException:
            return None
        q = tp.transform.rotation
        p = tp.transform.translation
        axis = _quat_to_R(q.x, q.y, q.z, q.w)[:, 1]          # head_joint1 axis: head_link1 y
        pivot = np.array([p.x, p.y, p.z])
        now = time.time()
        acc = [r for r in list(self.acc) if r[0] >= now - self.still_s]
        gyr = [r for r in list(self.gyr) if r[0] >= now - self.still_s]
        self.static = False
        if len(acc) > 20 and len(gyr) > 20:
            A = np.array(acc)[:, 1:]
            Gw = np.linalg.norm(np.array(gyr)[:, 1:], axis=1)
            a = A.mean(0)
            g = np.linalg.norm(a)
            self.static = bool(Gw.max() < self.still_rad and abs(g / 9.80665 - 1.0) < self.acc_tol)
            if self.static:
                v = R_bc @ (self.R_ic @ (a / g))                  # measured up, joint-predicted frame
                vp = v - (v @ axis) * axis
                up = self.up_base - (self.up_base @ axis) * axis
                raw = math.atan2(axis @ np.cross(vp, up), vp @ up)
                self.tilt_offaxis = math.degrees(math.asin(min(1.0, abs(v @ axis))) -
                                                 math.asin(min(1.0, abs(self.up_base @ axis))))
                self.tilt_raw = raw
                if abs(raw) <= self.tilt_max:
                    dt = now - self.tilt_t if self.tilt_t else 1e9
                    alpha = 1.0 - math.exp(-dt / self.tau) if self.tau > 0 else 1.0
                    self.tilt += alpha * (raw - self.tilt)
                    self.tilt_t = now
        return pivot, axis

    def _publish_status(self):
        now = time.time()
        if now - self.last_status < 0.5:
            return
        self.last_status = now
        self.status_pub.publish(String(data=json.dumps(dict(
            tilt_deg=math.degrees(self.tilt),
            raw_deg=None if self.tilt_raw is None else math.degrees(self.tilt_raw),
            static=self.static,
            age_s=None if self.tilt_t is None else now - self.tilt_t,
            offaxis_deg=self.tilt_offaxis))))

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
        if self.imu_on:
            tr, r = transform.transform.translation, transform.transform.rotation
            R_bc = _quat_to_R(r.x, r.y, r.z, r.w)
            t_bc = np.array([tr.x, tr.y, tr.z])
            pa = self._imu_tilt(R_bc)
            if pa is not None and self.tilt != 0.0:
                pivot, axis = pa
                Rc = _rotvec_to_R(axis * self.tilt)               # true = rot_at(pivot, axis, tilt) @ pred
                R_new = Rc @ R_bc
                t_new = Rc @ (t_bc - pivot) + pivot
                qx, qy, qz, qw = _R_to_quat(R_new)
                msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = \
                    (float(v) for v in t_new)
                msg.transform.rotation.x, msg.transform.rotation.y = float(qx), float(qy)
                msg.transform.rotation.z, msg.transform.rotation.w = float(qz), float(qw)
            self._publish_status()
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
