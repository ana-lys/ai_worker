#!/usr/bin/env python3
"""Record the D435i IMU + /joint_states while you move the head by hand (read-only).

Purpose: see how far the camera really moves (IMU) while the head encoders barely do
(backlash). Run it, push/tilt the head gently inside its play, hold each extreme still
for ~1 s, Ctrl-C when done.

Live line (10 Hz): head_joint1/2 [deg] + head_joint2 effort, camera pitch (looking
below horizon) and image roll from the JOINTS (TF base_link <- head_camera_frame) and
from the IMU (gravity, accel low-passed over --lpf-s), their difference, gyro rate,
and the session min/max of the difference.
  pitch/roll from gravity only see tilt -- a pure pan (head_joint1) is invisible to
  them; the gyro rate still shows it.

Saved on Ctrl-C to --out/backlash_<stamp>.npz:
  accel (N,4) t,x,y,z   gyro (N,4) t,x,y,z     [d435_camera optical frame, SI units]
  joints (M,1+3K) t, position[K], velocity[K], effort[K]   joint_names (K,)
  tf (L,5) t,qx,qy,qz,qw  (base_link <- head_camera_frame)
and a summary is printed: IMU-minus-joints pitch/roll at every still moment, and per
head_joint1 (tilt) encoder bin the spread of IMU pitch (= play the encoder does not see).
Per-axis backlash (tilt + pan): analyze_head_backlash.py on the saved .npz.

    python3 record_head_backlash.py
    python3 record_head_backlash.py --out ~/head_backlash --lpf-s 0.3
"""
import argparse
import datetime
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu, JointState

CAM, BASE = 'head_camera_frame', 'base_link'
HEAD = ('head_joint1', 'head_joint2')


def stamp_s(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def tilt_deg(up_cam):
    """(pitch: optical axis below horizon, roll: image horizon tilt) from the up vector in
    the optical frame (x right, y down, z forward)."""
    u = up_cam / np.linalg.norm(up_cam)
    return (math.degrees(math.asin(np.clip(-u[2], -1.0, 1.0))),
            math.degrees(math.atan2(u[0], -u[1])))


class Rec(rclpy.node.Node):
    def __init__(self):
        super().__init__('record_head_backlash')
        self.acc, self.gyr, self.tf_rows, self.js_rows = [], [], [], []
        self.js_names = None
        self.js_last = {}
        sd = qos_profile_sensor_data
        self.create_subscription(Imu, '/d435/accel/sample', self._acc, sd)
        self.create_subscription(Imu, '/d435/gyro/sample', self._gyr, sd)
        self.create_subscription(JointState, '/joint_states', self._js, 50)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)

    def _acc(self, m):
        a = m.linear_acceleration
        self.acc.append((stamp_s(m), a.x, a.y, a.z))

    def _gyr(self, m):
        w = m.angular_velocity
        self.gyr.append((stamp_s(m), w.x, w.y, w.z))

    def _js(self, m):
        if self.js_names is None:
            self.js_names = list(m.name)
        if list(m.name) != self.js_names:
            return
        n = len(m.name)
        vel = list(m.velocity) if len(m.velocity) == n else [math.nan] * n
        eff = list(m.effort) if len(m.effort) == n else [math.nan] * n
        self.js_rows.append((stamp_s(m), *m.position, *vel, *eff))
        self.js_last = {k: (p, e) for k, p, e in zip(m.name, m.position, eff)}

    def poll_tf(self):
        try:
            tr = self.tfb.lookup_transform(BASE, CAM, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        q = tr.transform.rotation
        t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        if not self.tf_rows or self.tf_rows[-1][0] != t:
            self.tf_rows.append((t, q.x, q.y, q.z, q.w))
        return Rot.from_quat([q.x, q.y, q.z, q.w])


def summarize(acc, gyr, tf, joints, names, lpf_s, still_dps):
    if len(acc) < 100 or len(tf) < 10 or len(joints) < 10:
        print('not enough data for a summary')
        return
    # still moments: 0.5 s windows with gyro peak below still_dps
    gt, gw = gyr[:, 0], np.degrees(np.linalg.norm(gyr[:, 1:], axis=1))
    # head_joint1 is the TILT (URDF axis y), the joint gravity can see; head_joint2 is pan
    i1 = names.index('head_joint1') if 'head_joint1' in names else None
    rows = []
    for t0 in np.arange(acc[0, 0] + 0.5, acc[-1, 0] - 0.5, 0.25):
        sel_g = (gt >= t0 - 0.25) & (gt < t0 + 0.25)
        if sel_g.sum() < 20 or gw[sel_g].max() > still_dps:
            continue
        sel_a = (acc[:, 0] >= t0 - lpf_s / 2) & (acc[:, 0] < t0 + lpf_s / 2)
        k = np.argmin(np.abs(tf[:, 0] - t0))
        j = np.argmin(np.abs(joints[:, 0] - t0))
        if sel_a.sum() < 10 or abs(tf[k, 0] - t0) > 0.2 or abs(joints[j, 0] - t0) > 0.2:
            continue
        p_i, r_i = tilt_deg(acc[sel_a, 1:].mean(0))
        p_t, r_t = tilt_deg(Rot.from_quat(tf[k, 1:]).inv().apply([0, 0, 1]))
        h1 = math.degrees(joints[j, 1 + i1]) if i1 is not None else math.nan
        rows.append((t0, h1, p_t, r_t, p_i, r_i))
    if not rows:
        print('no still moments found (hold the head still ~1 s at each position)')
        return
    R = np.array(rows)
    dp, dr = R[:, 4] - R[:, 2], R[:, 5] - R[:, 3]
    print(f'\n=== summary: {len(R)} still windows (0.5 s, gyro < {still_dps} deg/s) ===')
    print(f'IMU - joints pitch: mean {dp.mean():+.3f}  min {dp.min():+.3f}  max {dp.max():+.3f}  '
          f'span {np.ptp(dp):.3f} deg')
    print(f'IMU - joints roll : mean {dr.mean():+.3f}  min {dr.min():+.3f}  max {dr.max():+.3f}  '
          f'span {np.ptp(dr):.3f} deg')
    print(f'head_joint1 (tilt) encoder over these: {R[:, 1].min():+.3f} .. {R[:, 1].max():+.3f} deg '
          f'(span {np.ptp(R[:, 1]):.3f})')
    print('\nper head_joint1 (tilt) encoder bin (0.1 deg): IMU pitch spread = tilt play the encoder misses')
    print('  (pan moves also change camera pitch a little; analyze_head_backlash.py separates the axes)')
    bins = np.round(R[:, 1] / 0.1) * 0.1
    for b in np.unique(bins):
        s = bins == b
        print(f'  h1 {b:+7.2f} deg  n {s.sum():3d}  IMU pitch {R[s, 4].min():+8.3f} .. {R[s, 4].max():+8.3f} '
              f'(spread {np.ptp(R[s, 4]):.3f})  IMU-joints pitch {dp[s].mean():+.3f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', default='~/head_backlash')
    ap.add_argument('--lpf-s', type=float, default=0.2, help='accel averaging window for the live tilt [s]')
    ap.add_argument('--still-dps', type=float, default=1.0, help='summary: still = gyro peak below [deg/s]')
    a = ap.parse_args()
    out = os.path.expanduser(a.out)
    os.makedirs(out, exist_ok=True)

    rclpy.init()
    node = Rec()
    print('waiting for IMU / joint_states / TF ...')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 15 and not (node.acc and node.gyr and node.js_last and node.poll_tf()):
        rclpy.spin_once(node, timeout_sec=0.05)
    if not (node.acc and node.gyr and node.js_last):
        print('missing data: is realsense_udp_streamer (D435i IMU) and the robot running? '
              'check ROS_DOMAIN_ID')
        return
    for lst in (node.acc, node.gyr, node.js_rows, node.tf_rows):
        lst.clear()
    print('recording -- move the head by hand, hold each extreme ~1 s; Ctrl-C to stop and save\n')
    lo = {'p': math.inf, 'r': math.inf}
    hi = {'p': -math.inf, 'r': -math.inf}
    last_print = 0.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.005)
            Rbc = node.poll_tf()
            now = time.monotonic()
            if now - last_print < 0.1 or Rbc is None or len(node.acc) < 20 or not node.gyr:
                continue
            last_print = now
            t_last = node.acc[-1][0]
            win = [r[1:] for r in list(node.acc)[-200:] if r[0] >= t_last - a.lpf_s]
            p_i, r_i = tilt_deg(np.mean(win, axis=0))
            p_t, r_t = tilt_deg(Rbc.inv().apply([0, 0, 1]))
            dp, dr = p_i - p_t, r_i - r_t
            w = math.degrees(np.linalg.norm(node.gyr[-1][1:]))
            if w < a.still_dps:
                lo['p'], hi['p'] = min(lo['p'], dp), max(hi['p'], dp)
                lo['r'], hi['r'] = min(lo['r'], dr), max(hi['r'], dr)
            h1, _ = node.js_last.get('head_joint1', (math.nan, math.nan))
            h2, e2 = node.js_last.get('head_joint2', (math.nan, math.nan))
            print(f'\rh1 {math.degrees(h1):+7.3f} h2 {math.degrees(h2):+7.3f} deg (eff {e2:+7.1f}) | '
                  f'pitch J {p_t:+7.3f} I {p_i:+7.3f} d {dp:+6.3f} | roll J {r_t:+6.3f} I {r_i:+6.3f} '
                  f'd {dr:+6.3f} | gyro {w:5.1f}/s | still d-pitch {lo["p"]:+.2f}..{hi["p"]:+.2f} '
                  f'd-roll {lo["r"]:+.2f}..{hi["r"]:+.2f}   ', end='', flush=True)
    except KeyboardInterrupt:
        print()
    names = node.js_names or []
    acc, gyr = np.array(node.acc), np.array(node.gyr)
    tf, joints = np.array(node.tf_rows), np.array(node.js_rows)
    path = os.path.join(out, f'backlash_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.npz')
    np.savez_compressed(path, accel=acc, gyro=gyr, tf=tf, joints=joints, joint_names=np.array(names))
    print(f'saved {path}: accel {len(acc)}, gyro {len(gyr)}, joint_states {len(joints)}, tf {len(tf)}')
    summarize(acc, gyr, tf, joints, names, a.lpf_s, a.still_dps)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
