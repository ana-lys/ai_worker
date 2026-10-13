#!/usr/bin/env python3
"""pose_snapshot.py -- record named robot poses, averaged over the last 1 s.

Type a name and press Enter; the last --window seconds (default 1.0) of
  - EE pose of both arms in the base frame (solver achieved pose,
    /ik_solver/achieved_ee_pose_{l,r})
  - EE pose of both arms in marker_frame (each EE sample composed with the
    marker EKF base pose of that instant, /marker_ekf_odom)
  - /joint_states (every joint)
  - marker EKF base pose in marker_frame (/marker_ekf_odom), plus the
    filter's own reported std (sqrt of its covariance)
  - raw ArUco board detections (/head_camera/marker_board_pose, base_link)
are averaged, with sample std, and appended to a JSON file under that name.
If the robot was not still (EE or EKF std above the limits) nothing is
saved and you are asked to re-record: type the name again. Same when a
source has no data in the window (e.g. board not visible). Names already in
the file are refused (nothing is overwritten). Empty line = ignored,
Ctrl-C = quit.

    ros2 run ffw_spacemouse pose_snapshot.py [--file PATH] [--window 1.0]
        [--max-ee-std-mm 0.5] [--max-ee-rot-deg 0.1]
        [--max-ekf-std-mm 1.0] [--max-ekf-rot-deg 0.1]
"""
import argparse
import bisect
import json
import math
import os
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

DEFAULT_FILE = os.path.expanduser(
    '~/robotis_ws/src/ai_worker/ffw_spacemouse/data/pose_snapshots.json')
KEEP_S = 10.0   # history kept per source


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def quat_rotate(q, v):
    qv = np.array([v[0], v[1], v[2], 0.0])
    qc = np.array([-q[0], -q[1], -q[2], q[3]])
    return quat_mul(quat_mul(q, qv), qc)[:3]


def quat_mean(qs):
    """Sign-aligned normalized mean (fine for the small spread of a 3 s window)."""
    qs = np.array(qs, dtype=float)
    ref = qs[0]
    qs[np.dot(qs, ref) < 0] *= -1.0
    m = qs.mean(axis=0)
    return m / np.linalg.norm(m)


def quat_angle_spread_deg(qs, mean):
    """RMS angle (deg) of each quaternion from the mean."""
    d = np.abs(np.clip(np.dot(np.array(qs), mean), -1.0, 1.0))
    return float(np.degrees(np.sqrt(np.mean((2 * np.arccos(d)) ** 2))))


def rpy_xyz(q):
    """Extrinsic XYZ RPY, R = Rx(roll) Ry(pitch) Rz(yaw) (joy_hand's convention)."""
    x, y, z, w = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    pitch = math.asin(max(-1.0, min(1.0, r[0, 2])))
    yaw = math.atan2(-r[0, 1], r[0, 0])
    roll = math.atan2(-r[1, 2], r[2, 2])
    return [roll, pitch, yaw]


def pose_summary(ps, qs):
    p = np.array(ps)
    q = quat_mean(qs)
    return {
        'position': [round(float(v), 6) for v in p.mean(axis=0)],
        'quaternion_xyzw': [round(float(v), 6) for v in q],
        'rpy_xyz_rad': [round(v, 6) for v in rpy_xyz(q)],
        'position_std_mm': [round(float(v) * 1000, 3) for v in p.std(axis=0)],
        'rotation_spread_deg': round(quat_angle_spread_deg(qs, q), 4),
        'samples': len(ps),
    }


class Recorder(Node):
    def __init__(self):
        super().__init__('pose_snapshot')
        self.lock = threading.Lock()
        self.ee = {'left': deque(), 'right': deque()}   # (t, p, q) base frame
        self.ekf = deque()                               # (t, p, q) base in marker_frame
        self.js = deque()                                # (t, {name: pos})
        self.ekf_cov = deque()                           # (t, [std x, std y, std yaw])
        self.aruco = deque()                             # (t, p, q) board in base_link
        for arm, s in (('left', 'l'), ('right', 'r')):
            self.create_subscription(PoseStamped, f'/ik_solver/achieved_ee_pose_{s}',
                                     lambda m, a=arm: self.on_pose(self.ee[a], m.pose), 50)
        self.create_subscription(Odometry, '/marker_ekf_odom', self.on_ekf, 50)
        self.create_subscription(PoseStamped, '/head_camera/marker_board_pose',
                                 lambda m: self.on_pose(self.aruco, m.pose), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)

    def now(self):
        return time.monotonic()

    def _trim(self, dq):
        t = self.now() - KEEP_S
        while dq and dq[0][0] < t:
            dq.popleft()

    def on_pose(self, dq, pose):
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        with self.lock:
            dq.append((self.now(), p, q))
            self._trim(dq)

    def on_ekf(self, m):
        self.on_pose(self.ekf, m.pose.pose)
        c = m.pose.covariance
        with self.lock:
            self.ekf_cov.append((self.now(), [math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)),
                                              math.sqrt(max(c[35], 0.0))]))
            self._trim(self.ekf_cov)

    def on_js(self, m):
        with self.lock:
            self.js.append((self.now(), dict(zip(m.name, m.position))))
            self._trim(self.js)

    def snapshot(self, window):
        t0 = self.now() - window
        with self.lock:
            ee = {a: [x for x in d if x[0] >= t0] for a, d in self.ee.items()}
            ekf = [x for x in self.ekf if x[0] >= t0]
            ekf_all = list(self.ekf)
            js = [x for x in self.js if x[0] >= t0]
            ekf_cov = [x[1] for x in self.ekf_cov if x[0] >= t0]
            aruco = [x for x in self.aruco if x[0] >= t0]
        out, missing = {}, []

        out['ee_base'] = {}
        for arm, s in ee.items():
            if s:
                out['ee_base'][arm] = pose_summary([x[1] for x in s], [x[2] for x in s])
            else:
                out['ee_base'][arm] = None
                missing.append(f'ee {arm}')

        out['ekf_base_in_marker'] = (pose_summary([x[1] for x in ekf], [x[2] for x in ekf])
                                     if ekf else None)
        if not ekf:
            missing.append('marker EKF')
        if ekf_cov:
            c = np.array(ekf_cov).mean(axis=0)
            out['ekf_filter_std'] = {'x_mm': round(float(c[0]) * 1000, 3), 'y_mm': round(float(c[1]) * 1000, 3),
                                     'yaw_deg': round(math.degrees(float(c[2])), 4)}
        else:
            out['ekf_filter_std'] = None

        if aruco:
            out['aruco_board_in_base'] = pose_summary([x[1] for x in aruco], [x[2] for x in aruco])
            out['aruco_board_in_base']['rate_hz'] = round(len(aruco) / window, 2)
        else:
            out['aruco_board_in_base'] = None
            missing.append('ArUco detections')

        # EE in marker_frame: each EE sample with the latest EKF pose at or before it.
        out['ee_marker'] = {}
        times = [x[0] for x in ekf_all]
        for arm, s in ee.items():
            ps, qs = [], []
            for t, p, q in s:
                i = bisect.bisect_right(times, t) - 1
                if i < 0:
                    continue
                _, pb, qb = ekf_all[i]                 # T_marker<-base
                ps.append(pb + quat_rotate(qb, p))
                qs.append(quat_mul(qb, q))
            out['ee_marker'][arm] = pose_summary(ps, qs) if ps else None
            if not ps and s:
                missing.append(f'ee {arm} in marker (no EKF)')

        if js:
            names = sorted(set().union(*(x[1].keys() for x in js)))
            joints = {}
            for n in names:
                v = np.array([x[1][n] for x in js if n in x[1]])
                joints[n] = {'position': round(float(v.mean()), 6),
                             'std_deg': round(float(np.degrees(v.std())), 4)}
            out['joint_states'] = {'samples': len(js), 'joints': joints}
        else:
            out['joint_states'] = None
            missing.append('joint_states')
        return out, missing


def load(path):
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return json.load(f)


def save(path, data):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--file', default=DEFAULT_FILE, help=f'JSON output (default {DEFAULT_FILE})')
    ap.add_argument('--window', type=float, default=1.0, help='averaging window, s (default 1.0)')
    ap.add_argument('--max-ee-std-mm', type=float, default=0.5, help='re-record above this EE position std')
    ap.add_argument('--max-ee-rot-deg', type=float, default=0.1, help='re-record above this EE rotation spread')
    ap.add_argument('--max-ekf-std-mm', type=float, default=1.0, help='re-record above this EKF position std')
    ap.add_argument('--max-ekf-rot-deg', type=float, default=0.1, help='re-record above this EKF rotation spread')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Recorder()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    def spin():
        try:
            ex.spin()
        except Exception:   # ExternalShutdownException on rclpy shutdown
            pass
    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()

    print(f'pose_snapshot: averaging the last {args.window:g} s -> {args.file}')
    print(f'  re-record if EE std > {args.max_ee_std_mm} mm / {args.max_ee_rot_deg} deg or EKF > {args.max_ekf_std_mm} mm / {args.max_ekf_rot_deg} deg')
    print('type a name + Enter to record, Ctrl-C to quit')
    try:
        while True:
            name = input('name> ').strip()
            if not name:
                continue
            data = load(args.file)
            if name in data:
                print(f'  "{name}" already exists -- not recorded, pick another name')
                continue
            snap, missing = node.snapshot(args.window)
            too_high = []
            for arm in ('left', 'right'):
                for frame in ('ee_base', 'ee_marker'):
                    e = snap[frame][arm]
                    if e and max(e['position_std_mm']) > args.max_ee_std_mm:
                        too_high.append(f'{arm} {frame} pos std {max(e["position_std_mm"]):.2f} mm')
                    if e and e['rotation_spread_deg'] > args.max_ee_rot_deg:
                        too_high.append(f'{arm} {frame} rot {e["rotation_spread_deg"]:.3f} deg')
            k = snap['ekf_base_in_marker']
            if k and max(k['position_std_mm']) > args.max_ekf_std_mm:
                too_high.append(f'EKF pos std {max(k["position_std_mm"]):.2f} mm')
            if k and k['rotation_spread_deg'] > args.max_ekf_rot_deg:
                too_high.append(f'EKF rot {k["rotation_spread_deg"]:.3f} deg')
            if too_high or missing:
                why = '; '.join(too_high + [f'no {m} data' for m in missing])
                print(f'  NOT recorded -- {"robot not still" if too_high else "missing data"} ({why}).')
                print(f'  Hold still and type "{name}" again to re-record.')
                continue
            snap['recorded_at'] = datetime.now().isoformat(timespec='seconds')
            snap['window_s'] = args.window
            data[name] = snap
            save(args.file, data)
            msg = f'  saved "{name}" ({len(data)} in file)'
            for arm in ('left', 'right'):
                e = snap['ee_base'][arm]
                if e:
                    msg += f' | {arm} std {max(e["position_std_mm"]):.2f} mm'
            if snap['ekf_filter_std']:
                f = snap['ekf_filter_std']
                msg += f' | EKF sigma {f["x_mm"]:.1f}/{f["y_mm"]:.1f} mm {f["yaw_deg"]:.2f} deg'
            if snap['aruco_board_in_base']:
                a = snap['aruco_board_in_base']
                msg += f' | ArUco {a["rate_hz"]:.0f} Hz std {max(a["position_std_mm"]):.2f} mm'
            print(msg)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        rclpy.try_shutdown()   # wakes the spinning executor, then join it
        spinner.join(timeout=2.0)


if __name__ == '__main__':
    main()
