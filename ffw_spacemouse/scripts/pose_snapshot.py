#!/usr/bin/env python3
"""pose_snapshot.py -- record named robot poses, averaged over the last 3 s.

Type a name and press Enter; the last WINDOW_S seconds of
  - EE pose of both arms in the base frame (solver achieved pose,
    /ik_solver/achieved_ee_pose_{l,r})
  - EE pose of both arms in marker_frame (each EE sample composed with the
    marker EKF base pose of that instant, /marker_ekf_odom)
  - /joint_states (every joint)
  - marker EKF base pose in marker_frame (/marker_ekf_odom)
are averaged and appended to a JSON file under that name. Names already in
the file are refused (nothing is overwritten). Each source also stores its
sample count and std, so a snapshot taken while moving is easy to spot.
Empty line = ignored, Ctrl-C = quit.

    ros2 run ffw_spacemouse pose_snapshot.py [--file PATH] [--window 3.0]
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
        for arm, s in (('left', 'l'), ('right', 'r')):
            self.create_subscription(PoseStamped, f'/ik_solver/achieved_ee_pose_{s}',
                                     lambda m, a=arm: self.on_pose(self.ee[a], m.pose), 50)
        self.create_subscription(Odometry, '/marker_ekf_odom',
                                 lambda m: self.on_pose(self.ekf, m.pose.pose), 50)
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
    ap.add_argument('--window', type=float, default=3.0, help='averaging window, s (default 3.0)')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Recorder()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    print(f'pose_snapshot: averaging the last {args.window:g} s -> {args.file}')
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
            snap['recorded_at'] = datetime.now().isoformat(timespec='seconds')
            snap['window_s'] = args.window
            data[name] = snap
            save(args.file, data)
            msg = f'  saved "{name}" ({len(data)} in file)'
            for arm in ('left', 'right'):
                e = snap['ee_base'][arm]
                if e:
                    msg += f' | {arm} std {max(e["position_std_mm"]):.1f} mm'
            print(msg)
            if missing:
                print(f'  WARNING: no data in the window for: {", ".join(missing)}')
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        ex.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
