#!/usr/bin/env python3
"""connector_align_capture.py -- marker-frame repeatability run over several
base placements.

For each base placement (the first is where the robot stands; the rest are
random in a box around it, measured from the board) the RIGHT arm visits the
connector points a b c d, then d c b a:
  approach to a stand-off -STANDOFF along marker_frame x in front of the point (script
  moves, reference orientation), hand over to the SpaceMouse with roll/pitch/
  yaw locked -> you align the translation and press Enter -> a 1 s snapshot is
  recorded (re-record if not still) -> the script retracts -RETRACT along
  marker_frame x,
  slides to the next point and approaches again.
Between placements the arm retracts -BASE_RETRACT, the base drives to the next
placement, and the arm approaches the first point.

Targets (position + orientation per point) are the means of the a-d snapshots
already recorded with pose_snapshot.py (--targets, names like 001a .. 004d),
mapped through a LEVEL board frame (raw ArUco board origin xy + yaw in
base_link; the 2D marker EKF leaks the board's apparent tilt into x/y) with
the solver frame taken as base_link + --solver-offset (MJCF arm base 25.4 mm
+x of the URDF).

Snapshots are named automatically: B<placement>_<ad|da>_<point>, e.g. B2_ad_c,
and written to data/connector_align_<time>.json with the same fields as
pose_snapshot.py plus the base placement.

Arm: /spacemouse/<arm>/external_hold + /quest/right/ee_target_pose (absolute,
solver frame); release re-bases joy_hand on the achieved pose. Base: claims
/cmd_vel from joy_base through /logitech/base_active and drives closed loop on
/odom (one scale for the whole twist). Needs the plain unified launch
(use_teleop_core:=false): teleop_core ignores that claim.
The lift stays at the recorded level: before the first motion it is moved
there if needed (asks y/N first; solver qpos rail, both arms held) and hard-locked in the IK
(/teleop_locks), so the arm alone reaches every target; its lock state is
restored at the end. Ctrl-C stops the base, releases both arms, unlocks and
un-claims.

    ros2 run ffw_spacemouse connector_align_capture.py --plan-only
    ros2 run ffw_spacemouse connector_align_capture.py
"""
import argparse
import glob
import json
import math
import os
import re
import signal
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from pose_snapshot import Recorder, quat_mean  # noqa: E402

DATA_DIR = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_spacemouse/data')
POINTS = 'abcd'
RAIL_DT = 0.01
# right-arm joint ranges of the IK model (ffw_bg2_large_margin_no_gripper_joint.xml)
ARM_R_LIMITS = {'arm_r_joint1': (-2.35619, 0.78539), 'arm_r_joint2': (-3.14, 0.0),
                'arm_r_joint3': (-1.57, 1.57), 'arm_r_joint4': (-2.61, 1.047),
                'arm_r_joint5': (-1.57, 1.57), 'arm_r_joint6': (-1.57, 1.57),
                'arm_r_joint7': (-1.5804, 1.8201)}


def R_from_q(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def q_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0] * 4
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def slerp(q0, q1, u):
    q0, q1 = np.array(q0), np.array(q1)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def level_board(board_p, board_q):
    """Level board frame in base_link: (xy origin, yaw) from a 3D board pose."""
    R = R_from_q(board_q)
    return np.array(board_p[:2], dtype=float), math.atan2(R[1, 0], R[0, 0])


# ── targets from pose_snapshot.py data ───────────────────────────────────────
def load_targets(path, offset):
    data = json.load(open(path))
    pts = {}
    for name, s in data.items():
        m = re.fullmatch(r'\d+([abcd])', name)
        if not m or not s.get('aruco_board_in_base') or not s['ee_base'].get('right'):
            continue
        o, yaw = level_board(s['aruco_board_in_base']['position'], s['aruco_board_in_base']['quaternion_xyzw'])
        e = s['ee_base']['right']
        p = np.array(e['position']) - [offset, 0.0, 0.0]          # solver -> base_link
        Rl = Rz(yaw).T
        pl = np.append(Rl[:2, :2] @ (p[:2] - o), p[2])             # level board frame
        ql = q_from_R(Rl @ R_from_q(e['quaternion_xyzw']))
        pts.setdefault(m.group(1), []).append((pl, ql))
    missing = [c for c in POINTS if c not in pts]
    if missing:
        sys.exit(f'targets file {path} has no snapshots for points {missing} (names like 001a)')
    return {c: (np.mean([x[0] for x in v], axis=0), quat_mean([x[1] for x in v]), len(v))
            for c, v in pts.items()}


class Control(Node):
    def __init__(self):
        super().__init__('connector_align_capture')
        self.lock = threading.Lock()
        self.ach = {}
        self.odom = None
        self.odom_t = 0.0
        self.board = []            # (t, p, q) ArUco board in base_link
        self.cmd_others = 0
        self.publishing = False
        for s in ('l', 'r'):
            self.create_subscription(PoseStamped, f'/ik_solver/achieved_ee_pose_{s}',
                                     lambda m, s=s: self._ach(s, m), 20)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(PoseStamped, '/head_camera/marker_board_pose', self._board,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 20)
        self.js = {}
        self.create_subscription(JointState, '/joint_states', self._js, 20)
        self.locks_pub = self.create_publisher(String, '/teleop_locks', 10)
        self.goal_source_pub = self.create_publisher(String, '/teleop_goal_source', 10)
        self.rail_pub = self.create_publisher(JointState, '/qpos_rail', 10)
        self.list_joints = self.create_client(Trigger, '/ik_solver/list_joints')
        self.goal_pub = self.create_publisher(PoseStamped, '/quest/right/ee_target_pose', 10)
        self.hold_pub = {n: self.create_publisher(Bool, f'/spacemouse/{n}/external_hold', 10)
                         for n in ('left', 'right')}
        self.ee_lock_pub = self.create_publisher(String, '/ik_solver/ee_lock', 10)
        self.claim_pub = self.create_publisher(Bool, '/logitech/base_active', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hold = {'left': False, 'right': False}   # nothing held until the run starts
        self.goal = None           # (p, q) solver frame, re-sent while the right arm is held
        self.create_timer(0.05, self._keepalive)

    def _ach(self, s, m):
        p, q = m.pose.position, m.pose.orientation
        with self.lock:
            self.ach[s] = (time.monotonic(), np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.odom = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            self.odom_t = time.monotonic()

    def _board(self, m):
        p, q = m.pose.position, m.pose.orientation
        with self.lock:
            self.board.append((time.monotonic(), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
            self.board = self.board[-200:]

    def _js(self, m):
        with self.lock:
            self.js.update(zip(m.name, m.position))

    def joint(self, name):
        with self.lock:
            return self.js.get(name)

    def solver_joints(self, timeout=3.0):
        """{joint: hard-locked} from /ik_solver/list_joints, None if no answer."""
        if not self.list_joints.wait_for_service(timeout_sec=timeout):
            return None
        fut = self.list_joints.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)                  # the executor thread completes it
        if not fut.done():
            return None
        out = {}
        for line in fut.result().message.splitlines():
            m = re.match(r'\s*\[\d+\]\s*\[(.*?)\]\s*(\S+)\s*$', line)
            if m:
                out[m.group(2)] = m.group(1).strip() == 'LOCKED'
        return out

    def set_locked(self, name, locked):
        """Hard lock / unlock a joint in the IK solve (/teleop_locks toggles, so
        publish only when the state differs). Returns the state after."""
        st = self.solver_joints()
        if st is None or name not in st:
            return None
        if st[name] != locked:
            self.locks_pub.publish(String(data=name))
            for _ in range(20):
                time.sleep(0.05)
                st = self.solver_joints()
                if st and st.get(name) == locked:
                    break
        return st.get(name) if st else None

    def move_lift(self, target, speed, tol):
        """Ramp the real lift to `target` over the solver's qpos rail (every other
        solver joint held at its current position), then back to goal source
        "ee" (the solver resyncs to the real joints). Returns the final lift."""
        names = list(self.solver_joints() or {})
        with self.lock:
            now = dict(self.js)
        if not names or 'lift_joint' not in names or any(n not in now for n in names):
            return None
        hold = {n: now[n] for n in names}
        lift0 = now['lift_joint']
        n_steps = max(1, int(math.ceil(abs(target - lift0) / speed / RAIL_DT)))

        def send(v):
            hold['lift_joint'] = v
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(hold)
            js.position = [float(x) for x in hold.values()]
            self.rail_pub.publish(js)
        try:
            send(lift0)
            self.goal_source_pub.publish(String(data='rail'))
            for k in range(1, n_steps + 1):
                send(lift0 + (target - lift0) * k / n_steps)
                time.sleep(RAIL_DT)
            t_end = time.monotonic() + 10.0          # until the real lift is there
            while time.monotonic() < t_end and abs(self.joint('lift_joint') - target) > tol:
                send(target)
                time.sleep(RAIL_DT)
            for _ in range(25):
                send(target)
                time.sleep(RAIL_DT)
        finally:
            self.goal_source_pub.publish(String(data='ee'))
            time.sleep(1.0)                          # solver resyncs to the real joints
        return self.joint('lift_joint')

    def _cmd(self, m):
        if not self.publishing:
            self.cmd_others += 1

    def _keepalive(self):
        # external_hold must be re-sent (joy_hand / teleop timeout 1 s); the
        # held right arm also gets its goal re-sent so the solver keeps it.
        for n, on in self.hold.items():
            if on:
                self.hold_pub[n].publish(Bool(data=True))
        if self.hold['right'] and self.goal is not None:
            self.send_goal(*self.goal)

    def send_goal(self, p, q):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(v) for v in q)
        self.goal_pub.publish(m)

    def set_hold(self, arm, on):
        self.hold[arm] = on
        if not on:
            for _ in range(3):
                self.hold_pub[arm].publish(Bool(data=False))

    def achieved(self, s='r'):
        with self.lock:
            return self.ach.get(s)

    def board_level(self, window=0.6, min_n=3):
        """Mean level board frame over the last `window` s, or None."""
        t0 = time.monotonic() - window
        with self.lock:
            b = [x for x in self.board if x[0] >= t0]
        if len(b) < min_n:
            return None
        o = np.mean([x[1] for x in b], axis=0)
        q = quat_mean([x[2] for x in b])
        return level_board(o, q)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--targets', default=os.path.join(DATA_DIR, 'pose_snapshots.json'))
    ap.add_argument('--out', default=None, help='default data/connector_align_<time>.json')
    ap.add_argument('--placements', type=int, default=4, help='base placements incl. the start pose')
    ap.add_argument('--x-range', default='-0.10,0.05', help='base x box around the start [m]')
    ap.add_argument('--y-range', default='-0.10,0.10', help='base y box around the start [m]')
    ap.add_argument('--yaw-deg', type=float, default=10.0, help='base yaw box +- [deg]')
    ap.add_argument('--min-move', type=float, default=0.05, help='min distance between placements [m]')
    ap.add_argument('--standoff', type=float, default=0.015, help='approach stand-off in -x [m]')
    ap.add_argument('--retract', type=float, default=0.05, help='retract between points in -x [m]')
    ap.add_argument('--base-retract', type=float, default=0.10, help='retract before a base move [m]')
    ap.add_argument('--marker-yaw-offset-deg', type=float, default=90.0,
                    help='marker_frame = board frame turned by this (marker_pose_corrector board_yaw_offset_rad)')
    ap.add_argument('--solver-offset', type=float, default=0.0254, help='solver frame = base_link + this x [m]')
    ap.add_argument('--reach-x', default='0.30,0.70', help='right EE x limits for a placement [m]')
    ap.add_argument('--reach-y', default='-0.55,0.10', help='right EE y limits for a placement [m]')
    ap.add_argument('--max-board-turn-deg', type=float, default=15.0,
                    help='reject placements that turn the board bearing more than this')
    ap.add_argument('--arm-vmax', type=float, default=0.05)
    ap.add_argument('--settle-mm', type=float, default=6.0, help='arm counts as arrived within this [mm]')
    ap.add_argument('--settle-timeout', type=float, default=4.0, help='max wait for arrival [s]')
    ap.add_argument('--handover-pos-mm', type=float, default=6.0,
                    help='achieved pose must be this close to the goal before the hand-over [mm]')
    ap.add_argument('--handover-rot-deg', type=float, default=1.5,
                    help='... and this close in orientation [deg]')
    ap.add_argument('--arm-wmax-deg', type=float, default=15.0)
    ap.add_argument('--base-vmax', type=float, default=0.10)
    ap.add_argument('--base-wmax', type=float, default=0.15)
    ap.add_argument('--base-tol-m', type=float, default=0.01)
    ap.add_argument('--base-tol-deg', type=float, default=1.0)
    ap.add_argument('--window', type=float, default=1.0)
    ap.add_argument('--max-ee-std-mm', type=float, default=0.5)
    ap.add_argument('--max-ee-rot-deg', type=float, default=0.1)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--lift', type=float, default=None,
                    help='lift_joint level for the run [m]; default = mean of the targets file snapshots')
    ap.add_argument('--lift-tol', type=float, default=0.002)
    ap.add_argument('--lift-speed', type=float, default=0.02, help='lift ramp when it must be moved [m/s]')
    ap.add_argument('--plan-only', action='store_true', help='print targets and placements, no motion')
    a, ros_args = ap.parse_known_args()

    targets = load_targets(a.targets, a.solver_offset)
    if a.lift is None:
        lifts = [s['joint_states']['joints']['lift_joint']['position']
                 for s in json.load(open(a.targets)).values()
                 if (s.get('joint_states') or {}).get('joints', {}).get('lift_joint')]
        if not lifts:
            sys.exit('no lift_joint in the targets file -- pass --lift')
        a.lift = float(np.mean(lifts))
    rng = np.random.default_rng(a.seed)
    out = a.out or os.path.join(DATA_DIR, f'connector_align_{datetime.now():%Y%m%d_%H%M%S}.json')
    if os.path.exists(out):
        sys.exit(f'{out} exists -- not overwriting')

    rclpy.init(args=ros_args)
    ctl = Control()
    rec = Recorder()
    ex = SingleThreadedExecutor()
    ex.add_node(ctl)
    ex.add_node(rec)

    def spin():
        try:
            ex.spin()
        except Exception:
            pass
    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()

    def log(s):
        print(s, flush=True)

    def finish(msg=None):
        """Stop ROS cleanly (join the spin thread) and exit."""
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)
        sys.exit(msg)

    # ── wait for inputs ──────────────────────────────────────────────────────
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        if ctl.achieved() and ctl.odom and ctl.board_level():
            break
        time.sleep(0.1)
    if not ctl.achieved():
        finish('no /ik_solver/achieved_ee_pose_r -- is the unified teleop launch running?')
    if not ctl.board_level():
        finish('board not detected on /head_camera/marker_board_pose')

    o0, yaw0 = ctl.board_level()           # board seen from the START base pose

    def base_in_start(o, yaw):
        """Current base pose relative to the start base, from the board (dx, dy, dyaw)."""
        # T_start_board = T_start_cur T_cur_board -> T_start_cur = T_start_board inv(T_cur_board)
        dyaw = yaw0 - yaw
        R = Rz(dyaw)[:2, :2]
        t = o0 - R @ o
        return np.array([t[0], t[1], dyaw])

    def targets_in_base(o, yaw, rel=(0.0, 0.0, 0.0)):
        """Point targets in the solver frame for a base at `rel` from the CURRENT base."""
        # board in the (possibly moved) base: T_new_board = inv(T_cur_new) T_cur_board
        dx, dy, dyaw = rel
        Rn = Rz(-dyaw)[:2, :2]
        on = Rn @ (o - np.array([dx, dy]))
        yawn = yaw - dyaw
        res = {}
        for c, (pl, ql, _) in targets.items():
            pb = np.append(Rz(yawn)[:2, :2] @ pl[:2] + on, pl[2]) + [a.solver_offset, 0, 0]
            res[c] = (pb, q_from_R(Rz(yawn) @ R_from_q(ql)))
        return res, on, yawn

    xr = [float(v) for v in a.x_range.split(',')]
    yr = [float(v) for v in a.y_range.split(',')]
    rx = [float(v) for v in a.reach_x.split(',')]
    ry = [float(v) for v in a.reach_y.split(',')]

    def marker_x(board_yaw):
        """marker_frame x axis in the base frame: the approach direction (toward the
        connector row); stand-off / retract distances are measured along it."""
        ang = board_yaw + math.radians(a.marker_yaw_offset_deg)
        return np.array([math.cos(ang), math.sin(ang), 0.0])

    def feasible(rel_start):
        """rel_start = base pose relative to START; check reach + board bearing."""
        cur = base_in_start(o0, yaw0)                       # == 0 at plan time
        rel = rel_start - cur
        tb, on, yawn = targets_in_base(o0, yaw0, rel)
        mx = marker_x(yawn)
        for c, (p, _) in tb.items():
            for dxr in (a.standoff, a.retract, a.base_retract):
                q = p - dxr * mx
                if not (rx[0] <= q[0] <= rx[1] and ry[0] <= q[1] <= ry[1]):
                    return False
        turn = abs(math.degrees(math.atan2(on[1], on[0]) - math.atan2(o0[1], o0[0])))
        return turn <= a.max_board_turn_deg

    placements = [np.zeros(3)]
    tries = 0
    while len(placements) < a.placements and tries < 5000:
        tries += 1
        cand = np.array([rng.uniform(*xr), rng.uniform(*yr), math.radians(rng.uniform(-a.yaw_deg, a.yaw_deg))])
        if min(np.linalg.norm(cand[:2] - p[:2]) for p in placements) < a.min_move:
            continue
        if feasible(cand):
            placements.append(cand)
    if len(placements) < a.placements:
        finish(f'only {len(placements)} feasible placements found -- widen the box or reach limits')

    log(f'targets from {a.targets} (level board frame, solver offset {a.solver_offset * 1000:.1f} mm):')
    tb0, _, _ = targets_in_base(o0, yaw0)
    for c in POINTS:
        log(f'  {c}: {targets[c][2]} snapshots -> now at solver {np.round(tb0[c][0], 4)}')
    log(f'placements (relative to start, x y [m], yaw [deg]), {a.placements} total:')
    for i, p in enumerate(placements):
        log(f'  B{i + 1}: {p[0]:+.3f} {p[1]:+.3f} {math.degrees(p[2]):+.1f}')
    log(f'per placement: a b c d, d c b a; stand-off {a.standoff * 100:.1f} cm, retract {a.retract * 100:.0f} cm, '
        f'base retract {a.base_retract * 100:.0f} cm, all along marker_frame x '
        f'(now {np.round(marker_x(yaw0)[:2], 3)} in base); output {out}')
    lift_now, st = ctl.joint('lift_joint'), ctl.solver_joints()
    lift_locked0 = None if st is None else st.get('lift_joint')
    log(f'lift: now {lift_now if lift_now is None else round(lift_now, 4)} m, run level {a.lift:+.4f} m '
        f'(tol {a.lift_tol * 1000:.0f} mm), {"LOCKED" if lift_locked0 else "free"} in the IK -> '
        + ('will be locked there' if lift_now is not None and abs(lift_now - a.lift) <= a.lift_tol
           else f'will be moved {"?" if lift_now is None else f"{(a.lift - lift_now) * 100:+.1f} cm"} '
                f'(both arms ride along) and locked'))
    if a.plan_only:
        finish()
    if lift_locked0 is None:
        finish('no lift_joint lock state from /ik_solver/list_joints -- is ffw_ik_solver_teleop running?')
    if not ctl.odom:
        finish('no /odom')
    if 'teleop_core' in [n for n, _ in ctl.get_node_names_and_namespaces()]:
        finish('teleop_core is running: use the plain unified launch (use_teleop_core:=false), '
                 'teleop_core ignores the /logitech/base_active base claim')

    # ── motion helpers ───────────────────────────────────────────────────────
    def move_arm(p_goal, q_goal, settle_mm=None, timeout=None):
        settle_mm = a.settle_mm if settle_mm is None else settle_mm
        timeout = a.settle_timeout if timeout is None else timeout
        ach = ctl.achieved()
        p0, q0 = ach[1], ach[2]
        if ctl.goal is not None:
            p0, q0 = ctl.goal
        d = np.linalg.norm(p_goal - p0)
        rot = math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(q0, q_goal))))))
        dur = max(d / a.arm_vmax, rot / a.arm_wmax_deg, 0.3)
        ts = time.monotonic()
        while True:
            u = min(1.0, (time.monotonic() - ts) / dur)
            s = u * u * (3 - 2 * u)
            ctl.goal = (p0 + s * (p_goal - p0), slerp(q0, q_goal, s))
            ctl.send_goal(*ctl.goal)
            if u >= 1.0:
                break
            time.sleep(0.02)
        tw = time.monotonic()
        while time.monotonic() - tw < timeout:
            ach = ctl.achieved()
            if ach and np.linalg.norm(ach[1] - p_goal) * 1000 < settle_mm:
                return True
            time.sleep(0.05)
        log(f'  (arm settled to {np.linalg.norm(ctl.achieved()[1] - p_goal) * 1000:.1f} mm, continuing)')
        return False

    def take_arm():
        ach = ctl.achieved()
        ctl.goal = (ach[1], ach[2])
        ctl.set_hold('right', True)
        time.sleep(0.3)               # joy_hand goes silent first

    def give_arm(lock_axes=('roll', 'pitch', 'yaw')):
        ctl.set_hold('right', False)
        time.sleep(0.4)               # joy_hand re-bases on the achieved pose
        for ax in ('roll', 'pitch', 'yaw'):
            ctl.ee_lock_pub.publish(String(data=f'unlock {ax}_r'))
        time.sleep(0.1)
        for ax in lock_axes:
            ctl.ee_lock_pub.publish(String(data=f'lock {ax}_r'))   # captures this orientation

    def stop_base():
        for _ in range(3):
            ctl.cmd_pub.publish(Twist())
            time.sleep(0.02)

    def drive_rel(dx, dy, dyaw, gain=1.2):
        """Drive by (dx, dy, dyaw) in the CURRENT base frame, closed loop on /odom."""
        with ctl.lock:
            sx, sy, syaw = ctl.odom
        gx = sx + dx * math.cos(syaw) - dy * math.sin(syaw)
        gy = sy + dx * math.sin(syaw) + dy * math.cos(syaw)
        gyaw = syaw + dyaw
        limit = math.hypot(dx, dy) + 0.10
        ts = time.monotonic()
        ctl.publishing = True
        try:
            while True:
                with ctl.lock:
                    o, age = ctl.odom, time.monotonic() - ctl.odom_t
                if age > 1.0:
                    raise RuntimeError(f'odom stale ({age:.2f} s)')
                if math.hypot(gx - o[0], gy - o[1]) > limit:
                    raise RuntimeError('base overshoot guard tripped')
                ex_w, ey_w = gx - o[0], gy - o[1]
                eyaw = math.atan2(math.sin(gyaw - o[2]), math.cos(gyaw - o[2]))
                if math.hypot(ex_w, ey_w) < a.base_tol_m and abs(eyaw) < math.radians(a.base_tol_deg):
                    break
                if time.monotonic() - ts > 40:
                    raise RuntimeError('base move timed out')
                c, s_ = math.cos(o[2]), math.sin(o[2])
                ex_b, ey_b = c * ex_w + s_ * ey_w, -s_ * ex_w + c * ey_w
                # one scale for the whole twist (swerve steering targets stay put)
                k = min(gain, a.base_vmax / max(math.hypot(ex_b, ey_b), 1e-9),
                        a.base_wmax / max(abs(eyaw), 1e-9))
                tw = Twist()
                tw.linear.x, tw.linear.y, tw.angular.z = float(k * ex_b), float(k * ey_b), float(k * eyaw)
                ctl.cmd_pub.publish(tw)
                time.sleep(0.05)
        finally:
            stop_base()
            ctl.publishing = False

    def drive_to_placement(want):
        """Drive so the base ends at `want` (relative to start, measured from the board)."""
        ctl.claim_pub.publish(Bool(data=True))
        time.sleep(0.5)
        ctl.cmd_others = 0
        time.sleep(0.6)
        if ctl.cmd_others:
            raise RuntimeError(f'{ctl.cmd_others} /cmd_vel msgs from another node after claiming the base')
        try:
            for it in range(3):
                bl = ctl.board_level()
                if bl is None:
                    raise RuntimeError('board lost before the base move')
                cur = base_in_start(*bl)
                err = want - cur
                if np.linalg.norm(err[:2]) < 0.01 and abs(err[2]) < math.radians(1.0):
                    break
                # error expressed in the current base frame
                d = Rz(-cur[2])[:2, :2] @ err[:2]
                log(f'  base move {it + 1}: dx {d[0]:+.3f} dy {d[1]:+.3f} m, dyaw {math.degrees(err[2]):+.1f} deg')
                drive_rel(d[0], d[1], err[2])
                time.sleep(1.5)                       # let the board estimate settle
        finally:
            stop_base()
            for _ in range(3):
                ctl.claim_pub.publish(Bool(data=False))
                time.sleep(0.05)

    data = {}

    def record(name, placement):
        while True:
            input(f'  [{name}] align the translation with the SpaceMouse, then Enter ')
            snap, missing = rec.snapshot(a.window)
            bad = []
            for frame in ('ee_base', 'ee_marker'):
                e = snap[frame]['right']
                if e and max(e['position_std_mm']) > a.max_ee_std_mm:
                    bad.append(f'{frame} pos std {max(e["position_std_mm"]):.2f} mm')
                if e and e['rotation_spread_deg'] > a.max_ee_rot_deg:
                    bad.append(f'{frame} rot {e["rotation_spread_deg"]:.3f} deg')
            missing = [m for m in missing if 'left' not in m]
            if bad or missing:
                log(f'  NOT recorded ({"; ".join(bad + [f"no {m}" for m in missing])}) -- hold still, Enter again')
                continue
            bl = ctl.board_level()
            snap['recorded_at'] = datetime.now().isoformat(timespec='seconds')
            snap['window_s'] = a.window
            snap['placement_planned'] = [float(v) for v in placement]
            snap['placement_measured'] = ([float(v) for v in base_in_start(*bl)] if bl else None)
            snap['board_level_in_base'] = ({'origin_xy': [float(v) for v in bl[0]], 'yaw': bl[1]} if bl else None)
            data[name] = snap
            tmp = out + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=1)
            os.replace(tmp, out)
            e = snap['ee_base']['right']
            log(f'  saved {name} ({len(data)} total), std {max(e["position_std_mm"]):.2f} mm')
            return

    def approach_point(c, stand_x):
        """Target of point c, backed off `stand_x` along marker_frame -x."""
        bl = ctl.board_level()
        if bl is None:
            raise RuntimeError('board not visible')
        tb, _, _ = targets_in_base(*bl)
        p, q = tb[c]
        return p - stand_x * marker_x(bl[1]), q

    def at_goal():
        """(position error mm, rotation error deg) of the achieved right EE vs the goal."""
        ach, (pg, qg) = ctl.achieved(), ctl.goal
        dp = float(np.linalg.norm(ach[1] - pg)) * 1000
        dr = math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(ach[2], qg))))))
        return dp, dr

    def joints_at_limit(margin=0.03):
        out = []
        for n, (lo, hi) in ARM_R_LIMITS.items():
            v = ctl.joint(n)
            if v is not None and (v - lo < margin or hi - v < margin):
                out.append(f'{n} {v:+.3f} (range {lo:+.3f}..{hi:+.3f})')
        return out

    def approach(c, from_retract):
        """Approach point c: re-orient at the retract distance first (the arm has room
        there), then translate to the stand-off. Before the hand-over the achieved pose
        must match the goal; if the IK stalled (e.g. at a joint limit) back off and retry
        once, then ask. Returns 'ok', 'skip' or 'free' (align with yaw unlocked)."""
        p_r, q = approach_point(c, from_retract)
        move_arm(ctl.goal[0], q)                         # rotate in place
        move_arm(p_r, q)
        move_arm(*approach_point(c, a.standoff))
        for attempt in range(2):
            dp, dr = at_goal()
            if dp <= a.handover_pos_mm and dr <= a.handover_rot_deg:
                return 'ok'
            log(f'  arm off the goal by {dp:.1f} mm / {dr:.2f} deg'
                + (f'; at a joint limit: {", ".join(joints_at_limit())}' if joints_at_limit() else ''))
            if attempt == 0:
                log('  retry: back off, re-orient, approach again')
                p_r, q = approach_point(c, a.base_retract)
                move_arm(p_r, ctl.goal[1])
                move_arm(p_r, q)
                move_arm(*approach_point(c, a.standoff))
        while True:
            ans = input('  IK did not reach the goal: [s]kip this point, [a]lign it yourself '
                        '(yaw unlocked), [q]uit? ').strip().lower()
            if ans in ('s', 'a', 'q'):
                break
        if ans == 'q':
            raise KeyboardInterrupt
        return 'skip' if ans == 's' else 'free'

    def handle_signal(signum, frame):
        raise KeyboardInterrupt          # terminal closed / killed: run the same cleanup
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, handle_signal)

    # leftover EE locks from an earlier run that died without its cleanup
    for ax in ('roll', 'pitch', 'yaw'):
        ctl.ee_lock_pub.publish(String(data=f'unlock {ax}_r'))

    # ── run ──────────────────────────────────────────────────────────────────
    log('\nSAFETY: base and right arm move by themselves. E-stop in hand. Press Enter to start, Ctrl-C to abort.')
    input()
    def retract(dist):
        """Back the right EE off by `dist` along marker_frame -x from where it is now."""
        bl = ctl.board_level()
        if bl is None:
            raise RuntimeError('board not visible')
        p, q = ctl.goal
        move_arm(p - dist * marker_x(bl[1]), q)

    def freeze_lift():
        lift = ctl.joint('lift_joint')
        if lift is None:
            raise RuntimeError('no lift_joint in /joint_states')
        if abs(lift - a.lift) > a.lift_tol:
            zl, zr = ctl.achieved('l'), ctl.achieved('r')
            dz = a.lift - lift
            ans = input(f'  The lift is {dz * 100:+.1f} cm from the run level ({lift:+.4f} -> {a.lift:+.4f} m). '
                        f'Both arms ride along: left EE z {zl[1][2] + dz:.3f}, right EE z {zr[1][2] + dz:.3f} m '
                        f'after the move. Clear below/above the arms? Move the lift [y/N] ').strip().lower()
            if ans != 'y':
                raise RuntimeError('lift not moved (answer was not y)')
            log(f'  lift {lift:+.4f} -> {a.lift:+.4f} m (qpos rail, both arms held)')
            if ctl.set_locked('lift_joint', False) is not False:
                raise RuntimeError('could not unlock lift_joint in the IK')
            lift = ctl.move_lift(a.lift, a.lift_speed, a.lift_tol)
            if lift is None or abs(lift - a.lift) > a.lift_tol:
                ctl.set_locked('lift_joint', True)
                raise RuntimeError(f'lift did not reach {a.lift:+.4f} (now {lift})')
        if ctl.set_locked('lift_joint', True) is not True:
            raise RuntimeError('could not lock lift_joint in the IK')
        log(f'  lift {ctl.joint("lift_joint"):+.4f} m, LOCKED in the IK for the run')

    try:
        ctl.set_hold('left', True)        # left arm stays where it is for the whole run
        ctl.set_hold('right', True)
        time.sleep(0.3)
        freeze_lift()
        # start reference AFTER the lift is in place (a lift move shifts the board estimate)
        time.sleep(1.0)
        bl = ctl.board_level()
        if bl is None:
            raise RuntimeError('board not visible after the lift check')
        o0, yaw0 = bl
        take_arm()
        seq = [('ad', c) for c in POINTS] + [('da', c) for c in reversed(POINTS)]
        for bi, placement in enumerate(placements):
            log(f'\n== placement B{bi + 1}/{len(placements)}: {placement[0]:+.3f} {placement[1]:+.3f} m '
                f'{math.degrees(placement[2]):+.1f} deg')
            if bi > 0:
                retract(a.base_retract)                     # clear the board before the base moves
                drive_to_placement(placement)
                from_retract = a.base_retract
            else:
                retract(a.retract)                          # from wherever the arm starts
                from_retract = a.retract
            for i, (pas, c) in enumerate(seq):
                name = f'B{bi + 1}_{pas}_{c}'
                if i > 0:
                    retract(a.retract)                      # back off the previous connector
                    from_retract = a.retract
                res = approach(c, from_retract)
                if res == 'skip':
                    log(f'  skipped {name}')
                    continue
                give_arm(('roll', 'pitch') if res == 'free' else ('roll', 'pitch', 'yaw'))
                record(name, placement)
                take_arm()
        log(f'\ndone: {len(data)} snapshots in {out}')
    except (KeyboardInterrupt, EOFError):
        log('\nstopped')
    except RuntimeError as e:
        log(f'\nABORT: {e}')
    finally:
        stop_base()
        ctl.claim_pub.publish(Bool(data=False))
        ctl.goal = None
        ctl.set_hold('right', False)
        ctl.set_hold('left', False)
        for ax in ('roll', 'pitch', 'yaw'):
            ctl.ee_lock_pub.publish(String(data=f'unlock {ax}_r'))
        time.sleep(0.3)
        if ctl.set_locked('lift_joint', lift_locked0) == lift_locked0:
            log(f'lift lock restored ({"LOCKED" if lift_locked0 else "free"})')
        else:
            log('WARNING: could not restore the lift lock state -- check /ik_solver/list_joints')
        log('released both arms and the base; EE locks cleared')
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)


if __name__ == '__main__':
    main()
