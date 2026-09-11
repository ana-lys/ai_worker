#!/usr/bin/env python3
"""Records camera<->base_link and board-pose data to check whether the
head_camera_frame URDF mount offset (the last link in the head kinematic
chain -> camera) is correct.

Hypothesis under test: that offset is wrong, so the marker board's computed
pose in base_link SHIFTS as the head moves even though the board itself is
physically stationary -- a fixed extrinsic error rotates WITH the camera as
the head pans/tilts (T_baselink_board = T_baselink_camera * T_camera_board),
so it does not cancel out the way it would if the board were misdetected by
a constant amount instead.

Records one row per detection pass (~5 Hz, triggered by
/oakd/marker_board_pose_camera_frame arriving) to a CSV:
    t, head_joint1, head_joint2,
    cam_x, cam_y, cam_z, cam_qw, cam_qx, cam_qy, cam_qz,        (T_baselink_camera, /head_camera_tf, cached)
    cam2brd_x, ..., cam2brd_qz,                                  (T_camera_board, this sample)
    brd_base_x, ..., brd_base_qz                                 (T_baselink_board, /oakd/marker_board_pose, cached)

The last three columns are ALREADY the composed pose the running node
publishes -- included so you don't have to recompute the composition by
hand to see if it moved. head_joint1/2 let you correlate any shift
against how far the head actually moved.

Usage:
    ros2 run ffw_depthai record_camera_calibration_check.py
    # then sweep the head through a real range of motion while it runs
    # Ctrl-C to stop -- writes CSV + prints a stability summary

Optional params:
    --ros-args -p output_path:=/path/to/file.csv
"""

import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_calib_lib import CSV_HEADER, build_row  # noqa: E402


class CameraCalibrationCheckRecorder(Node):
    def __init__(self):
        super().__init__('record_camera_calibration_check')

        default_path = os.path.expanduser(
            f'~/camera_calib_check_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        self.declare_parameter('output_path', default_path)
        self.output_path = self.get_parameter('output_path').value

        self.latest_cam_tf = None      # /head_camera_tf, T_baselink_camera
        self.latest_joint_state = None  # /joint_states
        self.latest_board_base = None   # /oakd/marker_board_pose, T_baselink_board

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TransformStamped, '/head_camera_tf', self._on_cam_tf, transient_qos)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(
            PoseStamped, '/oakd/marker_board_pose', self._on_board_base,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Trigger: one CSV row per detection pass.
        self.create_subscription(
            PoseStamped, '/oakd/marker_board_pose_camera_frame', self._on_board_camera,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.rows = []
        self.t0 = None
        self.get_logger().info(
            f'Recording to {self.output_path} -- move the head through its '
            'range of motion now. Ctrl-C to stop and see the summary.'
        )

    def _on_cam_tf(self, msg: TransformStamped):
        self.latest_cam_tf = msg

    def _on_joint_state(self, msg: JointState):
        self.latest_joint_state = msg

    def _on_board_base(self, msg: PoseStamped):
        self.latest_board_base = msg

    def _on_board_camera(self, msg: PoseStamped):
        if self.latest_cam_tf is None or self.latest_board_base is None:
            return  # not enough data yet to make a useful row

        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now

        h1 = h2 = float('nan')
        if self.latest_joint_state is not None:
            names = self.latest_joint_state.name
            pos = self.latest_joint_state.position
            if 'head_joint1' in names:
                h1 = pos[names.index('head_joint1')]
            if 'head_joint2' in names:
                h2 = pos[names.index('head_joint2')]

        row = build_row(now - self.t0, h1, h2,
                         self.latest_cam_tf, msg, self.latest_board_base)
        self.rows.append(row)
        if len(self.rows) % 25 == 0:
            self.get_logger().info(f'{len(self.rows)} rows recorded...')

    def write_and_summarize(self):
        if not self.rows:
            self.get_logger().warn('No rows recorded -- nothing to write.')
            return

        header = CSV_HEADER
        os.makedirs(os.path.dirname(self.output_path) or '.', exist_ok=True)
        with open(self.output_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.rows)
        print(f'\nWrote {len(self.rows)} rows to {self.output_path}')

        def col(name):
            i = header.index(name)
            return [r[i] for r in self.rows]

        def rng(name):
            vals = [v for v in col(name) if not (isinstance(v, float) and math.isnan(v))]
            return (min(vals), max(vals), max(vals) - min(vals)) if vals else (float('nan'),) * 3

        h1lo, h1hi, h1rng = rng('head_joint1')
        h2lo, h2hi, h2rng = rng('head_joint2')
        print(f'head_joint1 swept: {h1lo:.4f} .. {h1hi:.4f}  (range {math.degrees(h1rng):.1f} deg)')
        print(f'head_joint2 swept: {h2lo:.4f} .. {h2hi:.4f}  (range {math.degrees(h2rng):.1f} deg)')

        print('\nboard-in-base_link stability (should be ~constant if the '
              'camera mount offset is correct):')
        for axis in ('brd_base_x', 'brd_base_y', 'brd_base_z'):
            lo, hi, r = rng(axis)
            print(f'  {axis}: {lo:.4f} .. {hi:.4f}  range={r*100:.1f} cm')

        if h1rng + h2rng < math.radians(5):
            print('\nWARNING: head barely moved (< 5 deg total) -- this run does '
                  'not meaningfully test the calibration. Re-run and actually '
                  'sweep the head through a real range of motion.')


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationCheckRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_and_summarize()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
