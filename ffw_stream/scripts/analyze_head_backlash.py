#!/usr/bin/env python3
"""Head backlash from a record_head_backlash.py .npz: how far the camera really turns
about each head joint axis that the encoders do not report.

Still poses: gyro magnitude (moving mean over --smooth-s) below --still-dps AND both
head encoder velocities (smoothed over --enc-smooth-s) below --enc-dps, for at least --min-still-s (edges trimmed).
High-velocity frames are dropped entirely.

Joint axes (URDF ffw_sg2_follower): head_joint1 = TILT about arm_base_link y
(= base_link y); head_joint2 = PAN about head_link1 z = Ry(q1) z in base_link.

Two independent estimates of the camera's error vs the joint-predicted pose (TF
base_link <- head_camera_frame, recorded):
  gyro     integrate the gyro (still-pose bias removed) between consecutive still
           poses; the rotation the encoders did NOT report accumulates as an offset
           trajectory; projected on each joint axis. Sees both joints.
  gravity  per still pose, the rotation taking the joint-predicted up to the IMU up,
           projected on the tilt axis. Drift-free, but blind to pan (head_joint2).
Backlash per joint = max - min of the offset about its axis over all still poses.

    python3 analyze_head_backlash.py ~/head_backlash/backlash_20261012_121149.npz
"""
import argparse
import math

import numpy as np
from scipy.spatial.transform import Rotation as Rot


def segments(mask, t, min_len, trim):
    out, i, n = [], 0, len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        a, b = t[i] + trim, t[j] - trim
        if b - a >= min_len:
            out.append((a, b))
        i = j + 1
    return out


def integrate(gt, gw, t0, t1):
    """Body rotation from t0 to t1 (camera frame), trapezoid over the real stamps (gaps bridged)."""
    sel = np.flatnonzero((gt >= t0) & (gt <= t1))
    R = Rot.identity()
    if len(sel) < 2:
        return R
    for k0, k1 in zip(sel[:-1], sel[1:]):
        R = R * Rot.from_rotvec(0.5 * (gw[k0] + gw[k1]) * (gt[k1] - gt[k0]))
    return R


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('npz')
    ap.add_argument('--still-dps', type=float, default=2.0, help='gyro magnitude below [deg/s]')
    ap.add_argument('--enc-dps', type=float, default=1.0, help='head encoder speed below [deg/s]')
    ap.add_argument('--smooth-s', type=float, default=0.1, help='gyro smoothing [s]')
    ap.add_argument('--enc-smooth-s', type=float, default=0.5, help='encoder speed smoothing [s]')
    ap.add_argument('--min-still-s', type=float, default=0.3)
    ap.add_argument('--trim-s', type=float, default=0.1)
    a = ap.parse_args()

    d = np.load(a.npz)
    acc, gyr, tf, js = d['accel'], d['gyro'], d['tf'], d['joints']
    names = list(d['joint_names'])
    K = len(names)
    i1, i2 = names.index('head_joint1'), names.index('head_joint2')
    gt, gw = gyr[:, 0], gyr[:, 1:]
    jt = js[:, 0]
    q1, q2 = js[:, 1 + i1], js[:, 1 + i2]

    # stillness on the gyro timeline
    w = np.degrees(np.linalg.norm(gw, axis=1))
    k = max(1, int(round(a.smooth_s * 200)))
    w_s = np.convolve(w, np.ones(k) / k, mode='same')
    v1 = np.degrees(np.abs(np.gradient(q1, jt)))
    v2 = np.degrees(np.abs(np.gradient(q2, jt)))
    # encoders step 0.088 deg: one count in 10 ms reads 9 deg/s, so smooth them longer
    k_j = max(1, int(round(a.enc_smooth_s * 100)))
    venc = np.convolve(np.maximum(v1, v2), np.ones(k_j) / k_j, mode='same')
    venc_g = np.interp(gt, jt, venc)
    still = (w_s < a.still_dps) & (venc_g < a.enc_dps)
    segs = segments(still, gt, a.min_still_s, a.trim_s)
    print(f'{a.npz}\n{gt[-1] - gt[0]:.1f} s recorded; {still.mean() * 100:.0f} % of gyro frames still; '
          f'{len(segs)} still poses (>= {a.min_still_s} s)')
    if len(segs) < 2:
        print('need at least 2 still poses')
        return
    bias = gw[still].mean(0)
    print(f'gyro residual bias (still frames): {np.degrees(bias).round(4)} deg/s')

    rows = []
    for s0, s1 in segs:
        tm = 0.5 * (s0 + s1)
        sa = (acc[:, 0] >= s0) & (acc[:, 0] <= s1)
        st = (jt >= s0) & (jt <= s1)
        kt = np.argmin(np.abs(tf[:, 0] - tm))
        rows.append(dict(t0=s0, t1=s1, tm=tm, up=acc[sa, 1:].mean(0), n_acc=int(sa.sum()),
                         q1=q1[st].mean(), q2=q2[st].mean(),
                         q1_sd=q1[st].std(), q2_sd=q2[st].std(),
                         Rbc=Rot.from_quat(tf[kt, 1:]), tf_dt=abs(tf[kt, 0] - tm)))

    def axes(q1v):
        ax1 = np.array([0.0, 1.0, 0.0])
        ax2 = Rot.from_rotvec(ax1 * q1v).apply([0.0, 0.0, 1.0])
        return ax1, ax2

    # gyro: offset trajectory O (camera error vs joints), in base_link, small-angle sum
    gb = gw - bias
    off_g = [np.zeros(3)]
    steps = []
    for r0, r1 in zip(rows[:-1], rows[1:]):
        Rg = integrate(gt, gb, r0['tm'], r1['tm'])
        Rj = r0['Rbc'].inv() * r1['Rbc']                     # joint-predicted body rotation
        e_cam = (Rj.inv() * Rg).as_rotvec()                  # missed rotation, camera frame i+1
        e_base = r1['Rbc'].apply(e_cam)
        off_g.append(off_g[-1] + e_base)
        ax1, ax2 = axes(r1['q1'])
        enc_d1, enc_d2 = math.degrees(r1['q1'] - r0['q1']), math.degrees(r1['q2'] - r0['q2'])
        steps.append((math.degrees(Rg.magnitude()), enc_d1, enc_d2,
                      math.degrees(e_base @ ax1), math.degrees(e_base @ ax2),
                      math.degrees(np.linalg.norm(e_base - (e_base @ ax1) * ax1 - (e_base @ ax2) * ax2))))
    off_g = np.array(off_g)

    # gravity: per pose, rotation joint-up -> IMU-up, in base_link
    off_up = []
    for r in rows:
        up_imu = r['Rbc'].apply(r['up'] / np.linalg.norm(r['up']))   # IMU up in base_link
        up_j = np.array([0.0, 0.0, 1.0])
        c = np.cross(up_j, up_imu)
        # the camera turned by +d (vs the joints) => the up it measures turns by -d
        off_up.append(-c / max(np.linalg.norm(c), 1e-12) * math.asin(min(1.0, np.linalg.norm(c))))
    off_up = np.array(off_up)

    print('\npose   t[s]    dur  head_joint1  head_joint2 [deg]   gyro offset about j1  j2 [deg]   '
          'gravity about j1 [deg]')
    p1g, p2g, p1u = [], [], []
    for i, r in enumerate(rows):
        ax1, ax2 = axes(r['q1'])
        p1g.append(math.degrees(off_g[i] @ ax1))
        p2g.append(math.degrees(off_g[i] @ ax2))
        p1u.append(math.degrees(off_up[i] @ ax1))
        print(f'{i:4d} {r["tm"] - gt[0]:6.1f} {r["t1"] - r["t0"]:5.1f}  {math.degrees(r["q1"]):+9.3f}  '
              f'{math.degrees(r["q2"]):+11.3f}            {p1g[-1]:+8.3f} {p2g[-1]:+8.3f}            {p1u[-1]:+8.3f}')
    p1g, p2g, p1u = map(np.array, (p1g, p2g, p1u))
    p1u_rel = p1u - p1u[0]

    print('\nsteps between consecutive still poses: gyro turn, encoder deltas, MISSED about j1 / j2 / other')
    for i, s in enumerate(steps):
        print(f'  {i:3d}->{i + 1:<3d} gyro {s[0]:7.3f}  enc j1 {s[1]:+7.3f} j2 {s[2]:+7.3f}  '
              f'missed j1 {s[3]:+7.3f} j2 {s[4]:+7.3f} other {s[5]:6.3f}')

    print('\n=== backlash (max - min of the unreported rotation over all still poses) ===')
    print(f'head_joint1 (tilt): gyro {np.ptp(p1g):.3f} deg   gravity {np.ptp(p1u):.3f} deg   '
          f'(gyro vs gravity, relative to pose 0: max |diff| {np.abs(p1g - p1u_rel).max():.3f})')
    print(f'head_joint2 (pan) : gyro {np.ptp(p2g):.3f} deg   (gravity cannot see pan)')
    st = np.array(steps)
    print(f'largest single missed step: j1 {np.abs(st[:, 3]).max():.3f} deg, j2 {np.abs(st[:, 4]).max():.3f} deg, '
          f'off-axis {st[:, 5].max():.3f} deg')
    q1s, q2s = np.degrees([r['q1'] for r in rows]), np.degrees([r['q2'] for r in rows])
    print(f'encoders over the still poses: j1 {q1s.min():+.3f}..{q1s.max():+.3f} (span {np.ptp(q1s):.3f}), '
          f'j2 {q2s.min():+.3f}..{q2s.max():+.3f} (span {np.ptp(q2s):.3f}) deg')


if __name__ == '__main__':
    main()
