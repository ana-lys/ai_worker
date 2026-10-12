#!/usr/bin/env python3
"""Verify the D435i IMU published by realsense_udp_streamer (read-only, moves nothing).

STATIC (default, keep the robot + head still for --secs):
  1. streams   rates of /d435/{accel,gyro}/sample and /d435/imu, frame_id, stamp age
  2. noise     gyro bias (should be ~0 at rest) and accel/gyro noise
  3. scale     |accel| vs standard gravity (D435i IMUs ship roughly calibrated;
               rs-imu-calibration fixes scale/bias if this fails)
  4. frame     d435_camera (the IMU frame) is the calibrated TF optical frame
               head_camera_frame: /head_camera/marker_board_pose must equal
               TF(base_link <- head_camera_frame) x /head_camera/marker_board_pose_camera_frame
  5. gravity   measured "up" (accel direction) vs the up the TF chain predicts in the
               camera (base_link z): the angle is the base_link tilt vs gravity plus
               the IMU's own alignment error
  6. board     board normal vs gravity (board tilt vs the real world) next to
               board normal vs base_link z (the ~1 deg the calibration left)

DYNAMIC (--dynamic): during --secs, turn the head (any teleop/joystick; base still).
  The camera's angular velocity from TF (base_link <- head_camera_frame, finite
  differences, camera frame) is compared with the gyro: best-fit rotation between
  them (should be ~0 deg if the IMU->color extrinsics and frame are right), scale
  (~1.0), residual and the time offset that lines them up.

    python3 verify_d435_imu.py                  # static, 5 s
    python3 verify_d435_imu.py --dynamic --secs 20
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu

G = 9.80665
CAM = 'head_camera_frame'
BASE = 'base_link'


def stamp_s(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def ang_deg(u, v):
    u, v = u / np.linalg.norm(u), v / np.linalg.norm(v)
    return math.degrees(math.atan2(np.linalg.norm(np.cross(u, v)), float(np.dot(u, v))))


def pose_T(p):
    T = np.eye(4)
    q = p.orientation
    T[:3, :3] = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def tf_T(tr):
    T = np.eye(4)
    q, t = tr.transform.rotation, tr.transform.translation
    T[:3, :3] = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class Verdict:
    def __init__(self):
        self.fails = 0

    def __call__(self, ok, label, detail):
        tag = 'INFO' if ok is None else ('PASS' if bool(ok) else 'FAIL')   # ok may be a numpy bool
        self.fails += tag == 'FAIL'
        print(f'  [{tag}] {label:10s} {detail}')


class Probe(rclpy.node.Node):
    def __init__(self):
        super().__init__('verify_d435_imu')
        self.acc, self.gyr, self.imu = [], [], []   # (recv_t, stamp_t, xyz)
        self.frames = set()
        self.board_cam = self.board_base = None
        self.tf_rows = []                           # (stamp_t, R base<-cam)
        sd = qos_profile_sensor_data
        self.create_subscription(Imu, '/d435/accel/sample', lambda m: self._imu(m, self.acc, 'a'), sd)
        self.create_subscription(Imu, '/d435/gyro/sample', lambda m: self._imu(m, self.gyr, 'g'), sd)
        self.create_subscription(Imu, '/d435/imu', lambda m: self._imu(m, self.imu, 'i'), sd)
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, '/head_camera/marker_board_pose_camera_frame',
                                 lambda m: setattr(self, 'board_cam', m), be)
        self.create_subscription(PoseStamped, '/head_camera/marker_board_pose',
                                 lambda m: setattr(self, 'board_base', m), be)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)

    def _imu(self, m, store, kind):
        self.frames.add(m.header.frame_id)
        v = m.angular_velocity if kind == 'g' else m.linear_acceleration
        store.append((time.time(), stamp_s(m), np.array([v.x, v.y, v.z])))

    def cam_T(self):
        try:
            tr = self.tfb.lookup_transform(BASE, CAM, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None, None
        return tf_T(tr), tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9


def collect(node, secs, dynamic):
    t0 = time.monotonic()
    last_tf = None
    while time.monotonic() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.005)
        if dynamic:
            T, ts = node.cam_T()
            if T is not None and ts != last_tf:
                node.tf_rows.append((ts, T[:3, :3]))
                last_tf = ts


def static_checks(node, v, a):
    secs = a.secs
    acc = np.array([r[2] for r in node.acc]) if node.acc else None
    gyr = np.array([r[2] for r in node.gyr]) if node.gyr else None
    print('\n1. streams')
    for name, rows, want in (('accel', node.acc, 50), ('gyro', node.gyr, 100), ('imu', node.imu, 100)):
        hz = len(rows) / secs
        age = np.median([r[0] - r[1] for r in rows]) * 1000 if rows else float('nan')
        v(hz >= want, name, f'{hz:6.1f} Hz received, recv - stamp median {age:+.1f} ms')
    v(node.frames == {'d435_camera'}, 'frame_id', f'{sorted(node.frames)}')
    if acc is None or gyr is None:
        print('no IMU data -- is realsense_udp_streamer running with a D435i?')
        return None
    print('\n2. noise / bias (robot must be still)')
    gb = gyr.mean(0)
    v(np.linalg.norm(gb) < math.radians(0.5), 'gyro bias',
      f'{np.degrees(gb).round(3)} deg/s (|b| {math.degrees(np.linalg.norm(gb)):.3f}, want < 0.5)')
    v(None, 'noise', f'gyro sd {np.degrees(gyr.std(0)).round(3)} deg/s, accel sd {acc.std(0).round(4)} m/s^2')
    moving = np.linalg.norm(gyr, axis=1).max() > math.radians(3)
    if moving:
        v(False, 'still', 'gyro saw > 3 deg/s: something moved, static numbers below are unreliable')
    print('\n3. accel scale')
    am = acc.mean(0)
    err = (np.linalg.norm(am) - G) / G * 100
    v(abs(err) < 2.0, '|accel|', f'{np.linalg.norm(am):.3f} m/s^2 = {err:+.1f} % vs g (want within 2 %)')
    return am


def frame_and_gravity(node, v, am):
    T_bc, _ = node.cam_T()
    # The detector composes marker_board_pose from the same TF, so this only proves the
    # board/IMU frame "d435_camera" is the TF optical frame head_camera_frame (naming),
    # not that either is calibrated.
    print('\n4. IMU frame == TF head_camera_frame (via the board)')
    if node.board_cam is None or node.board_base is None or T_bc is None:
        v(None, 'frame', 'no board pose / TF -- skipped (is the board in view?)')
    else:
        pred = T_bc @ pose_T(node.board_cam.pose)
        meas = pose_T(node.board_base.pose)
        dp = np.linalg.norm(pred[:3, 3] - meas[:3, 3]) * 1000
        dr = math.degrees(Rot.from_matrix(pred[:3, :3].T @ meas[:3, :3]).magnitude())
        v(dp < 5 and dr < 0.2, 'frame',
          f'board via TF vs published: {dp:.1f} mm, {dr:.3f} deg (frame_ids '
          f'{node.board_cam.header.frame_id} / {node.board_base.header.frame_id})')
    if T_bc is None or am is None:
        return
    R_bc = T_bc[:3, :3]
    up_meas = am / np.linalg.norm(am)                 # specific force at rest points up
    up_tf = R_bc.T @ np.array([0.0, 0.0, 1.0])        # base_link z in the camera
    print('\n5. gravity vs the TF chain (camera frame)')
    tilt = ang_deg(up_meas, up_tf)
    # decompose in base_link: where does the measured up lean?
    up_b = R_bc @ up_meas
    lean = np.degrees([math.atan2(up_b[0], up_b[2]), math.atan2(up_b[1], up_b[2])])
    v(tilt < 1.5, 'up angle', f'{tilt:.2f} deg between accel-up and base_link z '
      f'(= base tilt vs gravity + IMU alignment error)')
    v(None, 'lean', f'gravity-up seen in base_link leans x {lean[0]:+.2f}, y {lean[1]:+.2f} deg '
      f'(+x: up tips forward, i.e. base nose-down)')
    print('\n6. board vs gravity')
    if node.board_cam is None:
        v(None, 'board', 'no board pose -- skipped')
        return
    n_cam = pose_T(node.board_cam.pose)[:3, 2]          # board normal (its z) in the camera
    if np.dot(n_cam, up_meas) < 0:
        n_cam = -n_cam
    v(None, 'board/g', f'board normal vs gravity-up : {ang_deg(n_cam, up_meas):.2f} deg  (board tilt in the world)')
    v(None, 'board/base', f'board normal vs base_link z: {ang_deg(n_cam, up_tf):.2f} deg  (what the TF/board pose shows)')


def dynamic_check(node, v):
    print('\nD. gyro vs TF angular velocity (head motion)')
    rows = node.tf_rows
    if len(rows) < 20:
        v(False, 'tf', f'only {len(rows)} TF updates of {BASE} <- {CAM}')
        return
    gt = np.array([r[1] for r in node.gyr])
    gw = np.array([r[2] for r in node.gyr])

    def pairs(offset):
        W_tf, W_g = [], []
        for (t0, R0), (t1, R1) in zip(rows[:-1], rows[1:]):
            dt = t1 - t0
            if not 0.005 < dt < 0.2:
                continue
            w = Rot.from_matrix(R0.T @ R1).as_rotvec() / dt      # camera frame (body)
            sel = (gt + offset >= t0) & (gt + offset < t1)
            if sel.sum() == 0:
                continue
            W_tf.append(w)
            W_g.append(gw[sel].mean(0))
        return np.array(W_tf), np.array(W_g)

    best = None
    for off in np.arange(-0.15, 0.1501, 0.005):
        A, B = pairs(off)
        if len(A) < 20:
            continue
        r = np.linalg.norm(A - B, axis=1).mean()
        if best is None or r < best[0]:
            best = (r, off, A, B)
    if best is None:
        v(False, 'pairs', 'not enough overlapping gyro/TF samples')
        return
    _, off, A, B = best
    mov = np.linalg.norm(A, axis=1) > math.radians(5)
    if mov.sum() < 10:
        v(False, 'motion', f'only {mov.sum()} intervals above 5 deg/s -- move the head more during --secs')
        return
    A, B = A[mov], B[mov]
    H = B.T @ A                                   # Kabsch: R such that R @ gyro ~ tf
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    Rfit = Vt.T @ D @ U.T
    rot = math.degrees(Rot.from_matrix(Rfit).magnitude())
    scale = np.sum(A * B) / np.sum(B * B)
    res = np.degrees(np.linalg.norm(A - B, axis=1))
    corr = [np.corrcoef(A[:, k], B[:, k])[0, 1] if A[:, k].std() > 1e-3 else float('nan') for k in range(3)]
    v(None, 'samples', f'{len(A)} moving intervals, peak {math.degrees(np.linalg.norm(A, axis=1).max()):.0f} deg/s')
    v(abs(off) < 0.05, 'time offset', f'{1000 * off:+.0f} ms (gyro stamp + offset lines up with TF)')
    v(rot < 2.0, 'rotation', f'{rot:.2f} deg best-fit rotation gyro -> TF camera frame (want ~0)')
    v(abs(scale - 1) < 0.05, 'scale', f'{scale:.3f} (want 1.00)')
    v(None, 'residual', f'median {np.median(res):.2f} deg/s, per-axis corr {np.round(corr, 3)}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--secs', type=float, default=5.0)
    ap.add_argument('--dynamic', action='store_true', help='move the head during the capture')
    a = ap.parse_args()
    rclpy.init()
    node = Probe()
    print('waiting for TF / topics ...')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10 and (node.cam_T()[0] is None or not node.gyr):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.acc.clear(), node.gyr.clear(), node.imu.clear()
    print(f'capturing {a.secs:.0f} s' + (' -- MOVE THE HEAD NOW' if a.dynamic else ' -- keep everything still'))
    collect(node, a.secs, a.dynamic)
    v = Verdict()
    if a.dynamic:
        dynamic_check(node, v)
    else:
        am = static_checks(node, v, a)
        frame_and_gravity(node, v, am)
    print(f'\n{"all checks passed" if v.fails == 0 else f"{v.fails} check(s) FAILED"}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if v.fails else 0)


if __name__ == '__main__':
    main()
