#!/usr/bin/env python3
"""Actively sweeps head_joint1/head_joint2 through a systematic grid,
records camera<->board calibration data at each settled pose, and solves
for the head_camera_frame mount correction.

For every (head_joint1, head_joint2) grid point (n_steps per joint, so
n_steps^2 poses total):
  1. Command the head there via /head_controller/follow_joint_trajectory.
  2. Wait settle_time seconds for it to stop moving/vibrating.
  3. Marker-count quality gate, read from /oakd/apriltag_telemetry's
     num_tags (one AprilTag detection pass ~= one "frame" here):
       - num_tags >= 5 right after settling -> accept immediately.
       - else wait 1 more detection frame; if that one has >= 4 -> accept.
       - else wait 2 more detection frames; if that one has >= 3 -> accept.
       - else skip this pose entirely (no row recorded).
  4. On accept, record one row: /head_camera_tf (T_baselink_camera),
     /oakd/marker_board_pose_camera_frame (T_camera_board), /joint_states,
     and /oakd/marker_board_pose (T_baselink_board, for convenience) --
     the exact same schema record_camera_calibration_check.py uses.

At the end (including on Ctrl-C, using whatever was collected so far), it
writes the CSV and runs the same least-squares solver as
solve_camera_mount_correction.py to report the corrected transform.

Usage:
    ros2 run ffw_depthai sweep_and_solve_camera_calibration.py

Useful params (--ros-args -p name:=value):
    n_steps (int, default 10)          steps per joint (grid = n_steps^2 poses)
    settle_time (double, default 2.0)  seconds to wait after each move
    range_margin_frac (double, 0.05)   fraction of each joint's URDF range to
                                        trim off BOTH ends, to stay clear of
                                        the mechanical hard stops
    max_joint_vel (double, 0.5)        rad/s, used only to size move duration
    output_path (string)               CSV output path
    return_home (bool, default true)   move back to (0, 0) when done
"""

import math
import os
import re
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_calib_lib import (  # noqa: E402
    CSV_HEADER, build_row, format_solution_report, row_to_matrices,
    solve_mount_correction,
)

# From ffw_description/urdf/common/follower/ffw_follower_body.xacro <limit> tags.
HEAD_JOINT1_LIMITS = (-0.2317, 0.6951)
HEAD_JOINT2_LIMITS = (-0.35, 0.35)

NUM_TAGS_RE = re.compile(r'num_tags=(\d+)')


class SweepAndSolveCalibration(Node):

    def __init__(self):
        super().__init__('sweep_and_solve_camera_calibration')

        default_path = os.path.expanduser(
            f'~/camera_calib_sweep_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        self.declare_parameter('output_path', default_path)
        self.declare_parameter('n_steps', 10)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('range_margin_frac', 0.05)
        self.declare_parameter('max_joint_vel', 0.5)  # rad/s
        self.declare_parameter('min_move_duration', 1.0)
        self.declare_parameter('gate_frame_timeout', 5.0)
        self.declare_parameter('return_home', True)
        self.declare_parameter('action_topic', '/head_controller/follow_joint_trajectory')
        self.declare_parameter('head_joint1_lower', HEAD_JOINT1_LIMITS[0])
        self.declare_parameter('head_joint1_upper', HEAD_JOINT1_LIMITS[1])
        self.declare_parameter('head_joint2_lower', HEAD_JOINT2_LIMITS[0])
        self.declare_parameter('head_joint2_upper', HEAD_JOINT2_LIMITS[1])

        gp = self.get_parameter
        self.output_path = gp('output_path').value
        self.n_steps = int(gp('n_steps').value)
        self.settle_time = float(gp('settle_time').value)
        self.range_margin_frac = float(gp('range_margin_frac').value)
        self.max_joint_vel = float(gp('max_joint_vel').value)
        self.min_move_duration = float(gp('min_move_duration').value)
        self.gate_frame_timeout = float(gp('gate_frame_timeout').value)
        self.return_home = bool(gp('return_home').value)
        action_topic = gp('action_topic').value
        self.j1_lo = float(gp('head_joint1_lower').value)
        self.j1_hi = float(gp('head_joint1_upper').value)
        self.j2_lo = float(gp('head_joint2_lower').value)
        self.j2_hi = float(gp('head_joint2_upper').value)

        # Cached latest data.
        self.latest_cam_tf = None          # /head_camera_tf, T_baselink_camera
        self.latest_joint_state = None      # /joint_states
        self.latest_board_base = None       # /oakd/marker_board_pose, T_baselink_board
        self.latest_board_camera = None     # /oakd/marker_board_pose_camera_frame, T_camera_board
        self.latest_num_tags = 0
        self.telemetry_seq = 0
        self.cur_h1 = 0.0
        self.cur_h2 = 0.0

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(TransformStamped, '/head_camera_tf', self._on_cam_tf, transient_qos)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(PoseStamped, '/oakd/marker_board_pose', self._on_board_base, best_effort)
        self.create_subscription(PoseStamped, '/oakd/marker_board_pose_camera_frame',
                                  self._on_board_camera, best_effort)
        self.create_subscription(StringMsg, '/oakd/apriltag_telemetry', self._on_telemetry, best_effort)

        self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)

        self.rows = []
        self.t0 = None

    # ---- subscriptions ----------------------------------------------------
    def _on_cam_tf(self, msg):
        self.latest_cam_tf = msg

    def _on_joint_state(self, msg):
        self.latest_joint_state = msg
        names = msg.name
        if 'head_joint1' in names:
            self.cur_h1 = msg.position[names.index('head_joint1')]
        if 'head_joint2' in names:
            self.cur_h2 = msg.position[names.index('head_joint2')]

    def _on_board_base(self, msg):
        self.latest_board_base = msg

    def _on_board_camera(self, msg):
        self.latest_board_camera = msg

    def _on_telemetry(self, msg):
        m = NUM_TAGS_RE.search(msg.data)
        if m:
            self.latest_num_tags = int(m.group(1))
            self.telemetry_seq += 1

    # ---- helpers ------------------------------------------------------------
    def spin_seconds(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, end - time.time())))

    def wait_new_telemetry(self, count, timeout):
        start_seq = self.telemetry_seq
        start_t = time.time()
        while self.telemetry_seq < start_seq + count:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_t > timeout:
                return False
        return True

    def build_waypoints(self):
        h1_vals = [self.j1_lo + (self.j1_hi - self.j1_lo) * self.range_margin_frac
                   + i * (self.j1_hi - self.j1_lo) * (1 - 2 * self.range_margin_frac)
                   / max(1, self.n_steps - 1)
                   for i in range(self.n_steps)]
        h2_vals = [self.j2_lo + (self.j2_hi - self.j2_lo) * self.range_margin_frac
                   + i * (self.j2_hi - self.j2_lo) * (1 - 2 * self.range_margin_frac)
                   / max(1, self.n_steps - 1)
                   for i in range(self.n_steps)]
        waypoints = []
        for i, h1 in enumerate(h1_vals):
            seq = h2_vals if i % 2 == 0 else list(reversed(h2_vals))
            for h2 in seq:
                waypoints.append((h1, h2))
        return waypoints

    def send_head_goal(self, h1, h2):
        if not self.action_client.server_is_ready():
            self.get_logger().info('Waiting for head_controller action server...')
            self.action_client.wait_for_server()

        dist = math.hypot(h1 - self.cur_h1, h2 - self.cur_h2)
        duration = max(self.min_move_duration, dist / max(1e-3, self.max_joint_vel))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['head_joint1', 'head_joint2']
        pt = JointTrajectoryPoint()
        pt.positions = [h1, h2]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start = DurationMsg(
            sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        goal.trajectory.points = [pt]

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('  head goal rejected or timed out sending')
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 5.0)

    def check_marker_gate(self):
        """Returns True (accept + record) or False (skip this pose)."""
        n0 = self.latest_num_tags
        if n0 >= 5:
            return True
        if not self.wait_new_telemetry(1, self.gate_frame_timeout):
            self.get_logger().warn(f'  no telemetry after settle (n={n0}) -- skipping')
            return False
        n1 = self.latest_num_tags
        if n1 >= 4:
            return True
        if not self.wait_new_telemetry(2, self.gate_frame_timeout):
            self.get_logger().warn(f'  telemetry stalled (n={n1}) -- skipping')
            return False
        n2 = self.latest_num_tags
        if n2 >= 3:
            return True
        self.get_logger().warn(f'  too few markers ({n0}->{n1}->{n2}) -- skipping')
        return False

    def record_row(self):
        if self.latest_cam_tf is None or self.latest_board_camera is None:
            self.get_logger().warn('  missing cam_tf or board_camera pose -- cannot record')
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        row = build_row(now - self.t0, self.cur_h1, self.cur_h2,
                         self.latest_cam_tf, self.latest_board_camera, self.latest_board_base)
        self.rows.append(row)
        self.get_logger().info(f'  recorded (num_tags={self.latest_num_tags}, '
                                f'{len(self.rows)} rows so far)')

    # ---- main sequence ------------------------------------------------------
    def run_sweep(self):
        self.get_logger().info(
            f'head_joint1 range [{self.j1_lo:.4f}, {self.j1_hi:.4f}] rad, '
            f'head_joint2 range [{self.j2_lo:.4f}, {self.j2_hi:.4f}] rad, '
            f'{self.n_steps} steps each ({self.n_steps**2} poses), '
            f'{self.range_margin_frac*100:.0f}% margin trimmed off each end.')
        self.get_logger().info('Make sure the AprilTag board is in view. Starting in 3s '
                                '(Ctrl-C now to abort)...')
        self.spin_seconds(3.0)

        waypoints = self.build_waypoints()
        total = len(waypoints)
        accepted = skipped = 0
        for idx, (h1, h2) in enumerate(waypoints):
            self.get_logger().info(
                f'[{idx+1}/{total}] -> head_joint1={h1:+.3f} head_joint2={h2:+.3f} rad')
            self.send_head_goal(h1, h2)
            self.spin_seconds(self.settle_time)
            if self.check_marker_gate():
                self.record_row()
                accepted += 1
            else:
                skipped += 1

        self.get_logger().info(f'Sweep complete: {accepted} accepted, {skipped} skipped '
                                f'out of {total} poses.')
        if self.return_home:
            self.get_logger().info('Returning head to (0, 0)...')
            self.send_head_goal(0.0, 0.0)

    def finish(self):
        if not self.rows:
            self.get_logger().warn('No rows recorded -- nothing to write or solve.')
            return
        os.makedirs(os.path.dirname(self.output_path) or '.', exist_ok=True)
        import csv
        with open(self.output_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            w.writerows(self.rows)
        print(f'\nWrote {len(self.rows)} rows to {self.output_path}')

        if len(self.rows) < 6:
            print('Fewer than 6 samples -- not enough to solve a 6-DOF correction. '
                  'Re-run with more steps / fewer skips.')
            return

        rows_as_dicts = [dict(zip(CSV_HEADER, r)) for r in self.rows]
        matrices = [row_to_matrices(r) for r in rows_as_dicts]
        A = [m[0] for m in matrices]
        B = [m[1] for m in matrices]
        sol = solve_mount_correction(A, B, reg_weight=50.0)
        print()
        print(format_solution_report(sol))
        print(f'\n(Re-run standalone any time with: '
              f'python3 solve_camera_mount_correction.py {self.output_path})')


def main(args=None):
    rclpy.init(args=args)
    node = SweepAndSolveCalibration()
    try:
        node.run_sweep()
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted -- writing/solving with whatever was collected so far.')
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
