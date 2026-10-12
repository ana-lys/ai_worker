#!/usr/bin/env python3
"""Head-camera mount calibration: drive base / lift / head, capture still,
sharp AprilTag-board samples, and solve the camera_calibration_link ->
head_camera_frame mount.

CAPTURE (default)
  For every base placement (start, +-dx, +-dy, +-yaw; closed loop on /odom),
  every lift height, and every image target in a grid, the head is servoed
  so the BOARD CENTRE lands on that image target. The target grid is fitted
  to the board's projected size each time, so the whole board (every tag
  corner, projected from the current pose + board layout + intrinsics) stays
  inside --edge-margin of the image: no frame is wasted on a board clipped at
  the edge, and the samples still span the usable image area.
  A sample is taken only when everything is still: head/lift joint
  velocities and base twist ~0 for --still-s, then --passes consecutive
  detection passes, each with >= --min-tags tags and reprojection
  < --max-reproj px, whose poses agree within --max-spread-mm / -deg (a
  moving or blurred image fails that).

  Each sample line (JSONL) holds: base placement group, odom, all joints,
  TF base_link->camera_calibration_link, /head_camera_tf, and every pass
  from <ns>/apriltag_detections (pose + every tag's raw pixel corners).
  The header line holds camera_info and the current mount.

  SAFETY: prints the plan and waits for Enter (--yes skips, --plan-only
  only prints). Base motion needs a live /odom and no other active /cmd_vel
  publisher; it is capped at --base-vmax / --base-wmax, aborts on stale odom
  or overshoot, and zero velocity is always sent on exit / Ctrl-C. The lift
  carries the arms -- make sure they have room to move down. Everything is
  returned to its start pose at the end.

  Each sample also waits for the D435i gyro to be still (--imu-still-s window:
  |mean| < --imu-still-dps and peak < --imu-peak-dps; lets the ~1 deg head play
  settle; a timeout skips the step) and records the IMU mean (accel/gyro, d435_camera
  frame) and base_link -> head_link1/head_link2 (joint pivots + axes).
  Detection passes whose frame the gyro saw turning (--det-still-dps over
  --det-motion-pre-s before the capture stamp) are dropped and replaced by later
  passes, so no smeared detection reaches a sample.

SOLVE
  --solve FILE.jsonl: bundle-style fit of the mount M (6 DOF) plus one board
  pose per base placement, minimising the reprojection of every recorded tag
  corner (robust soft-L1, 1 px). Samples with head_links get per-sample head
  play (tilt about head_link1 y, pan about head_link2 z, through the pivots;
  prior --play-sigma-deg). Samples with a still IMU add gravity residuals and
  solve the IMU->camera rotation correction + floor tilt (soft priors). Prints M
  as static_transform_publisher args, per-axis 1-sigma, per-group / worst-sample
  residuals, and writes <input>_solve.json (mount + IMU terms) for
  head_camera_tf_bridge's runtime IMU tilt correction.

    ros2 run ffw_depthai head_cam_calib_capture.py --plan-only
    ros2 run ffw_depthai head_cam_calib_capture.py            # capture
    ros2 run ffw_depthai head_cam_calib_capture.py --no-base  # head+lift only
    ros2 run ffw_depthai head_cam_calib_capture.py --solve ~/head_cam_calib_*.jsonl
"""
import argparse
import collections
import itertools
import json
import math
import os
import re
import sys
import threading
import time

import numpy as np

# ── AprilTag 25h9 board layout: copy of ffw_stream board_pose_detector.hpp ──
# (id, size, x, y, z, rx, ry, rz) -- board frame = tag 1; axis-angle rotation.
BOARD_LAYOUT = [
    (1, 0.057, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (2, 0.057, 0.18355213452308197, -0.0005495590273141514, 0.0,
     0.003392793153333883, -0.001652768831466606, 0.005596125526348661),
    (3, 0.057, -0.002604290279393214, 0.18262499145172836, 0.0,
     0.0022404711209055793, -0.0017038086815047686, 0.01313233381601511),
    (4, 0.057, -0.1821974196064706, -0.0016526647265309707, 0.0,
     -0.0005940458783676034, 0.003384338795834879, 0.0077421718916435315),
    (5, 0.041, 0.05777201673670586, -0.19155240115435038, 0.0,
     0.0024879033811299974, -0.0017303780467724601, 0.020033850859068666),
    (7, 0.041, -0.0903143682118519, -0.09851627871582525, 0.0,
     -0.003121444688450445, 0.002343849457630141, -0.009063486113326337),
    (8, 0.041, 0.08846322550535259, -0.09928499260033768, 0.0,
     -0.0041147398871153335, 0.0019800553991998655, 0.010351065748505018),
    (9, 0.041, -0.058080502405420564, -0.19258298833310175, 0.0,
     -0.0011850492009372472, 0.0012848023973317715, 0.000540310454424208),
    (20, 0.015, -0.033553076106144185, -0.2921062756094909, 0.025,
     -0.0010910218728025943, 0.0030283245402257302, 0.00023589369988523032),
    (25, 0.105, -0.17414292814679452, 0.16938160622453025, 0.0,
     -0.002828623660624173, 0.001109427942824287, 0.01576257580464725),
    (30, 0.015, 0.04296011055731651, -0.29273904961469743, 0.025,
     -0.008045401913811596, 0.01695414816661725, -0.01750620945015386),
    (31, 0.015, -0.0852440748546845, -0.28330018814973335, 0.01,
     -0.0344126233894834, -0.015019776036531377, -0.005325592433057143),
    (32, 0.015, 0.09416774058800953, -0.2862785785077211, 0.01,
     -0.022274917428141635, -0.013421948142595262, -0.017686629412183158),
]

class NotStill(RuntimeError):
    """wait_still timed out; walk/grid samples skip the step instead of aborting."""


HEAD_TILT_LIMITS = (-0.2317, 0.6951)   # head_joint1, +about y = nose down
HEAD_PAN_LIMITS = (-0.35, 0.35)        # head_joint2, +about z = look left
LIFT_LIMITS = (-0.5, 0.0)              # lift_joint, 0 = top


def rotvec_to_R(rv):
    rv = np.asarray(rv, float)
    a = np.linalg.norm(rv)
    if a < 1e-12:
        return np.eye(3)
    k = rv / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def R_to_rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    if abs(a - math.pi) < 1e-6:
        w, v = np.linalg.eigh(R)
        k = v[:, np.argmax(w)]
        return k * a
    return a / (2 * math.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def quat_to_R(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_from(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def planar_T(x, y, yaw):
    c, s_ = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s_, 0, x], [s_, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def board_corners():
    """{id: 4x3 corners in board frame}, apriltag p[0..3] order."""
    out = {}
    for tid, size, x, y, z, rx, ry, rz in BOARD_LAYOUT:
        h = size / 2
        local = np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]])
        out[tid] = (rotvec_to_R([rx, ry, rz]) @ local.T).T + np.array([x, y, z])
    return out


BOARD = board_corners()
BOARD_ALL = np.vstack(list(BOARD.values()))
BOARD_CENTER = BOARD_ALL.mean(axis=0)


# ════════════════════════════════════════════════════════════════════ CAPTURE
def capture(a):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Imu, JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectoryPoint
    import tf2_ros

    class Cap(Node):
        def __init__(self):
            super().__init__('head_cam_calib_capture')
            self.lock = threading.Lock()
            self.js = {}
            self.jv = {}
            self.odom = None
            self.odom_t = 0.0
            self.cam_info = None
            self.cam_tf = None
            self.dets = []          # (recv_mono, dict)
            self.cmd_vel_others = 0
            self.acc = collections.deque(maxlen=2000)   # (stamp_s, x, y, z) D435i, d435_camera frame
            self.gyr = collections.deque(maxlen=2000)
            self.motion_drops = set()                   # id() of passes dropped for camera motion
            be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(JointState, '/joint_states', self.on_js, 20)
            self.create_subscription(Odometry, '/odom', self.on_odom, 20)
            self.create_subscription(CameraInfo, f'{a.ns}/camera_info', self.on_ci, latched)
            self.create_subscription(TransformStamped, '/head_camera_tf', self.on_tf, latched)
            self.create_subscription(String, f'{a.ns}/apriltag_detections', self.on_det, be)
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
            self.imu_subs = [
                self.create_subscription(Imu, a.imu_ns + '/accel/sample', self.on_acc, qos_profile_sensor_data),
                self.create_subscription(Imu, a.imu_ns + '/gyro/sample', self.on_gyr, qos_profile_sensor_data)]
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.head_ac = ActionClient(self, FollowJointTrajectory,
                                        '/head_controller/follow_joint_trajectory')
            self.lift_ac = ActionClient(self, FollowJointTrajectory,
                                        '/lift_controller/follow_joint_trajectory')
            # lift through ffw_ik_solver_teleop (it streams the lift itself, overriding
            # lift_controller goals): its qpos rail, as ik_solver_cli / ee_handover_episodes
            self.goal_source_pub = self.create_publisher(String, '/teleop_goal_source', 10)
            self.rail_pub = self.create_publisher(JointState, '/qpos_rail', 10)
            self.locks_pub = self.create_publisher(String, '/teleop_locks', 10)
            self.hold_pubs = [self.create_publisher(Bool, f'/spacemouse/{n}/external_hold', 10)
                              for n in ('left', 'right')]
            self.list_joints = self.create_client(Trigger, '/ik_solver/list_joints')
            # claim the base from joy_base (SpaceMouse) the way the Logitech base
            # teleop does: it stays silent on /cmd_vel while this is true
            self.base_claim_pub = self.create_publisher(Bool, '/logitech/base_active', 10)
            self.base_claimed = False
            self.tfbuf = tf2_ros.Buffer()
            self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
            self.publishing = False

        def on_js(self, m):
            with self.lock:
                for i, n in enumerate(m.name):
                    self.js[n] = m.position[i]
                    if i < len(m.velocity):
                        self.jv[n] = m.velocity[i]

        def on_odom(self, m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            tw = m.twist.twist
            with self.lock:
                self.odom = dict(x=p.x, y=p.y, yaw=yaw, vx=tw.linear.x, vy=tw.linear.y,
                                 wz=tw.angular.z)
                self.odom_t = time.monotonic()

        def on_ci(self, m):
            with self.lock:
                self.cam_info = dict(w=m.width, h=m.height, K=list(m.k), D=list(m.d),
                                     frame=m.header.frame_id)

        def on_tf(self, m):
            t, r = m.transform.translation, m.transform.rotation
            with self.lock:
                self.cam_tf = dict(t=[t.x, t.y, t.z], q=[r.w, r.x, r.y, r.z],
                                   stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

        def on_det(self, m):
            try:
                d = json.loads(m.data)
            except ValueError:
                return
            with self.lock:
                self.dets.append((time.monotonic(), d))
                self.dets = self.dets[-200:]

        def on_acc(self, m):
            v = m.linear_acceleration
            with self.lock:
                self.acc.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, v.x, v.y, v.z))

        def on_gyr(self, m):
            v = m.angular_velocity
            with self.lock:
                self.gyr.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, v.x, v.y, v.z))

        def imu_window(self, secs):
            """Mean accel / gyro over the newest `secs` (None if no fresh IMU data)."""
            # newest samples only: copying the whole 2000-sample deques at 50 Hz
            # starved the executor (0.3 s stalls -> "odom stale" abort, 2026-10-12)
            n = int(secs * 300) + 20
            with self.lock:
                acc = list(itertools.islice(reversed(self.acc), n))[::-1]
                gyr = list(itertools.islice(reversed(self.gyr), n))[::-1]
            if not acc or not gyr or time.time() - acc[-1][0] > 0.5:
                return None
            t_end = acc[-1][0]
            A = np.array([r[1:] for r in acc if r[0] >= t_end - secs])
            G = np.array([r[1:] for r in gyr if r[0] >= t_end - secs])
            if len(A) < 10 or len(G) < 10:
                return None
            return dict(accel=A.mean(0).tolist(), accel_sd=A.std(0).tolist(), gyro=G.mean(0).tolist(),
                        gyro_peak_dps=float(np.degrees(np.linalg.norm(G, axis=1).max())),
                        gyro_mean_dps=float(np.degrees(np.linalg.norm(G.mean(0)))),
                        n_accel=len(A), n_gyro=len(G), window_s=secs, t_end=t_end)

        def gyro_peak(self, t0, t1):
            """Peak |gyro| [deg/s] over stamps [t0, t1]; None when the IMU does not cover t1."""
            with self.lock:
                if not self.gyr or self.gyr[-1][0] < t1:
                    return None
                gyr = []
                for r in reversed(self.gyr):          # newest first; stop past t0
                    if r[0] < t0:
                        break
                    gyr.append(r)
            w = [math.sqrt(r[1] * r[1] + r[2] * r[2] + r[3] * r[3]) for r in gyr if r[0] <= t1]
            return float(np.degrees(max(w))) if w else None

        def on_cmd_vel(self, m):
            if not self.publishing:
                self.cmd_vel_others += 1

        def snap(self):
            with self.lock:
                return dict(js=dict(self.js), jv=dict(self.jv), odom=dict(self.odom or {}),
                            odom_age=time.monotonic() - self.odom_t if self.odom else None,
                            cam_tf=self.cam_tf)

        def lookup(self, parent, child):
            try:
                t = self.tfbuf.lookup_transform(parent, child, rclpy.time.Time())
            except Exception as e:  # noqa: BLE001
                return None, str(e)
            tr, r = t.transform.translation, t.transform.rotation
            return dict(t=[tr.x, tr.y, tr.z], q=[r.w, r.x, r.y, r.z]), None

    import signal
    from rclpy.signals import SignalHandlerOptions
    # rclpy's own handlers would shut ROS down under the main loop (SIGTERM
    # left it running blind); instead both signals raise KeyboardInterrupt
    # here, so the finally block stops the base and returns to start with ROS
    # still alive.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def _raise(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)
    node = Cap()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    log = lambda *s: print('[calib]', *s, flush=True)  # noqa: E731

    def shutdown():
        # stop the executor thread before tearing rclpy down -- exiting with it
        # still spinning aborts the interpreter. The 200+ Hz IMU subscriptions go
        # first and in-flight callbacks get a moment to drain; otherwise teardown
        # raced them ("cannot use Destroyable because destruction was requested").
        if node.base_claimed:
            for _ in range(5):
                node.base_claim_pub.publish(Bool(data=False))
                time.sleep(0.02)
            node.base_claimed = False
        for sub in node.imu_subs:
            node.destroy_subscription(sub)
        time.sleep(0.2)
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    def bail(msg):
        log(msg)
        shutdown()
        sys.exit(1)

    def stop_base():
        node.publishing = True
        for _ in range(5):
            node.cmd_pub.publish(Twist())
            time.sleep(0.02)

    def send_traj(ac, joints, positions, duration, wait=True):
        if not ac.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f'action server for {joints} not available')
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        g.trajectory.points = [pt]
        # rclpy race: with the executor spinning on another thread the goal
        # response can arrive before the client registers the request ("Ignoring
        # unexpected goal response") and the future never completes. Resending
        # the same target is harmless, so retry.
        gh = None
        for attempt in range(4):
            fut = ac.send_goal_async(g)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                time.sleep(0.01)
            if fut.done():
                gh = fut.result()
                break
            log(f'goal response lost for {joints} -- resending ({attempt + 1}/3)')
        if gh is None:
            raise RuntimeError(f'goal not accepted for {joints}')
        if not gh.accepted:
            raise RuntimeError(f'goal rejected for {joints}')
        res = gh.get_result_async()
        if not wait:
            return res, joints, duration
        wait_traj((res, joints, duration))

    def wait_traj(h):
        res, joints, duration = h
        t0 = time.monotonic()
        while not res.done():
            if time.monotonic() - t0 > duration + 10:
                raise RuntimeError(f'{joints} did not finish')
            time.sleep(0.02)

    def move_head(tilt, pan):
        s = node.snap()['js']
        d = max(abs(tilt - s.get('head_joint1', tilt)), abs(pan - s.get('head_joint2', pan)))
        send_traj(node.head_ac, ['head_joint1', 'head_joint2'], [tilt, pan],
                  max(a.head_min_s, d / a.head_vmax))

    class LiftRail:
        """Lift via ffw_ik_solver_teleop's qpos rail while it runs: every solver joint
        streamed at its position when the capture started (arms frozen in joint space,
        so the upper body rides the lift as in SpaceMouse BASE mode), only lift_joint
        ramped at --lift-vmax; SpaceMouse external hold on. stop() restores goal source
        "ee", releases the hold and puts back a lift lock it removed."""
        DT = 0.01

        def __init__(self):
            self.on, self.relock, self.timer = False, False, None
            self.mtx = threading.Lock()

        def solver_joints(self, timeout=3.0):
            if not node.list_joints.wait_for_service(timeout_sec=timeout):
                return None
            fut = node.list_joints.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < timeout:
                time.sleep(0.01)
            if not fut.done():
                return None
            out = {}
            for line in fut.result().message.splitlines():
                m = re.match(r'\s*\[\d+\]\s*\[(.*?)\]\s*(\S+)\s*$', line)
                if m:
                    out[m.group(2)] = m.group(1).strip() == 'LOCKED'
            return out

        def start(self):
            st = self.solver_joints()
            if not st or 'lift_joint' not in st:
                log('lift: no ffw_ik_solver_teleop (/ik_solver/list_joints) -- lift_controller action')
                return False
            js = node.snap()['js']
            missing = [n for n in st if n not in js]
            if missing:
                raise RuntimeError(f'lift rail: no joint state for {missing}')
            if st['lift_joint']:
                node.locks_pub.publish(String(data='lift_joint'))     # toggles
                for _ in range(40):
                    time.sleep(0.05)
                    st2 = self.solver_joints()
                    if st2 and not st2['lift_joint']:
                        break
                else:
                    raise RuntimeError('lift rail: could not unlock lift_joint in the IK solver')
                self.relock = True
            self.hold = {n: float(js[n]) for n in st}
            self.sp = self.target = self.hold['lift_joint']
            self.last_hold = 0.0
            self._send()
            node.goal_source_pub.publish(String(data='rail'))
            self.timer = node.create_timer(self.DT, self._tick)
            self.on = True
            log(f'lift: IK solver qpos rail ({len(self.hold)} joints held, lift '
                f'{"unlocked for the run" if self.relock else "already unlocked"})')
            return True

        def _send(self):
            m = JointState()
            m.header.stamp = node.get_clock().now().to_msg()
            m.name = list(self.hold)
            m.position = list(self.hold.values())
            node.rail_pub.publish(m)

        def _tick(self):
            with self.mtx:
                step = a.lift_vmax * self.DT
                self.sp += max(-step, min(step, self.target - self.sp))
                self.hold['lift_joint'] = self.sp
            self._send()
            now = time.monotonic()
            if now - self.last_hold >= 0.2:
                for p in node.hold_pubs:
                    p.publish(Bool(data=True))
                self.last_hold = now

        def set(self, z):
            with self.mtx:
                self.target = float(min(LIFT_LIMITS[1], max(LIFT_LIMITS[0], z)))

        def wait(self, tol=0.003, timeout=None):
            """Until the ramp is done and the REAL lift is within tol; False on timeout."""
            dist = abs(self.target - self.sp)
            t_end = time.monotonic() + (timeout or dist / a.lift_vmax + 5.0)
            while time.monotonic() < t_end:
                cur = node.snap()['js'].get('lift_joint')
                if self.sp == self.target and cur is not None and abs(cur - self.target) < tol:
                    return True
                time.sleep(0.02)
            return False

        def stop(self):
            if not self.on:
                return
            if self.timer is not None:
                node.destroy_timer(self.timer)
            node.goal_source_pub.publish(String(data='ee'))
            for p in node.hold_pubs:
                p.publish(Bool(data=False))
            time.sleep(1.0)                                  # solver resyncs to the real joints
            if self.relock:
                node.locks_pub.publish(String(data='lift_joint'))
            self.on = False
            log('lift: rail released (goal source ee' + (', lift re-locked)' if self.relock else ')'))

    rail = LiftRail()

    def move_lift(z):
        if rail.on:
            rail.set(z)
            if not rail.wait():
                raise RuntimeError(f'lift did not reach {z:+.3f} (at {node.snap()["js"].get("lift_joint")})')
            return
        cur = node.snap()['js'].get('lift_joint', z)
        send_traj(node.lift_ac, ['lift_joint'], [z], max(0.8, abs(z - cur) / a.lift_vmax))

    def imu_still():
        """Camera still per the D435i gyro over --imu-still-s (True when there is no IMU)."""
        w = node.imu_window(a.imu_still_s)
        return w is None or (w['gyro_mean_dps'] < a.imu_still_dps and w['gyro_peak_dps'] < a.imu_peak_dps)

    def imu_record():
        w = node.imu_window(a.imu_still_s)
        if w is not None and not (w['gyro_mean_dps'] < a.imu_still_dps and w['gyro_peak_dps'] < a.imu_peak_dps):
            w['moving'] = True
        return w

    def head_links():
        """base_link -> head_link1 / head_link2 (joint pivots + axes for the play model)."""
        return {k: node.lookup('base_link', k)[0] for k in ('head_link1', 'head_link2')}

    def wait_still(use_base):
        t_still = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15:
            s = node.snap()
            moving = any(abs(s['jv'].get(j, 0.0)) > a.still_joint_vel
                         for j in ('head_joint1', 'head_joint2', 'lift_joint'))
            if use_base and s['odom']:
                o = s['odom']
                moving |= abs(o['vx']) > 0.005 or abs(o['vy']) > 0.005 or abs(o['wz']) > 0.01
            if moving:
                t_still = None
            elif t_still is None:
                t_still = time.monotonic()
            elif time.monotonic() - t_still >= a.still_s and imu_still():
                # (monotonic, wall) -- wall time is compared with the detector's
                # cap_stamp (frame capture time on the system clock)
                return time.monotonic(), time.time()
            time.sleep(0.02)
        w = node.imu_window(a.imu_still_s)
        imu = (f'; gyro mean {w["gyro_mean_dps"]:.2f} peak {w["gyro_peak_dps"]:.2f} deg/s' if w else '')
        raise NotStill(f'robot never came to rest in 15 s (joints/odom moving: {moving}{imu})')

    def passes_after(t_min, n, timeout):
        """n detection passes whose FRAME was captured after t_min = (mono, wall):
        by cap_stamp when the detector provides it, else by receive time.
        A pass whose frame the gyro saw moving (peak >= --det-still-dps over
        cap_stamp - --det-motion-pre-s .. cap_stamp + 0.02 s) is dropped; one the
        IMU does not cover yet waits for it (kept if it never does: no IMU)."""
        mono, wall = t_min if isinstance(t_min, tuple) else (t_min, None)

        def moved(d, tr):
            cs = d.get('cap_stamp')
            if not cs:
                return False
            if 'cap_gyro_dps' not in d:
                pk = node.gyro_peak(cs - a.det_motion_pre_s, cs + 0.02)
                if pk is None and time.monotonic() - tr < 0.5:
                    return True                 # IMU not there yet: not counted for now
                d['cap_gyro_dps'] = pk
            pk = d['cap_gyro_dps']
            return pk is not None and pk >= a.det_still_dps

        def fresh_list():
            with node.lock:
                dets = list(node.dets)
            out = []
            for tr, d in dets:
                cs = d.get('cap_stamp')
                if not (cs > wall if cs and wall is not None else tr > mono):
                    continue
                if moved(d, tr):
                    if d.get('cap_gyro_dps') is not None:
                        node.motion_drops.add(id(d))
                    continue
                out.append(d)
            return out
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            fresh = fresh_list()
            if len(fresh) >= n:
                return fresh[:n]
            time.sleep(0.02)
        return fresh_list()

    def board_px(pose):
        """(centre_uv, all-corner uv Nx2, depth) for a camera-frame board pose."""
        ci = node.cam_info
        K = np.array(ci['K']).reshape(3, 3)
        R = quat_to_R(*pose['q'])
        t = np.array(pose['t'])
        P = (R @ np.vstack([BOARD_CENTER, BOARD_ALL]).T).T + t
        uv = (K @ (P / P[:, 2:3]).T).T[:, :2]
        return uv[0], uv[1:], P[0, 2]

    def latest_pose(t_min, timeout=2.5):
        for d in reversed(passes_after(t_min, 1, timeout) or []):
            if d.get('pose'):
                return d
        return None

    def walk_target(pose, rng):
        """Walk image target for the board centre: the principal point (camera looking
        straight at the board) plus a uniform random offset, +-R_u in u and +-R_v in v
        with R_v = R_u * H / W (tilt range smaller than pan by the 1280:720 aspect).
        R_u = --walk-aim-frac of the room the board has around the centre before it
        leaves the --edge-margin; R_v is also clipped to that room."""
        ci = node.cam_info
        W, H = ci['w'], ci['h']
        K = np.array(ci['K']).reshape(3, 3)
        u0, v0 = K[0, 2], K[1, 2]
        c, uv, _ = board_px(pose)
        half_u = max(c[0] - uv[:, 0].min(), uv[:, 0].max() - c[0])
        half_v = max(c[1] - uv[:, 1].min(), uv[:, 1].max() - c[1])
        mu, mv = a.edge_margin * W, a.edge_margin * H
        room_u = max(0.0, min(u0 - (mu + half_u), (W - mu - half_u) - u0))
        room_v = max(0.0, min(v0 - (mv + half_v), (H - mv - half_v) - v0))
        r_u = a.walk_aim_frac * room_u
        r_v = min(r_u * H / W, room_v)
        return u0 + rng.uniform(-r_u, r_u), v0 + rng.uniform(-r_v, r_v)

    def targets_for(pose):
        """Image targets for the board centre, fitted so the whole board stays
        inside the edge margin wherever its centre is put."""
        ci = node.cam_info
        W, H = ci['w'], ci['h']
        c, uv, _ = board_px(pose)
        half_u = max(c[0] - uv[:, 0].min(), uv[:, 0].max() - c[0])
        half_v = max(c[1] - uv[:, 1].min(), uv[:, 1].max() - c[1])
        mu, mv = a.edge_margin * W, a.edge_margin * H
        lo_u, hi_u = mu + half_u, W - mu - half_u
        lo_v, hi_v = mv + half_v, H - mv - half_v
        if lo_u > hi_u or lo_v > hi_v:
            return []          # board too big to fit with margins at this distance
        n = a.grid
        # span < 1 pulls the grid toward the middle of the allowed range:
        # edge targets need head angles past the joint limits
        lo_s = 0.5 - a.grid_span / 2
        fr = [0.5] if n == 1 else [lo_s + a.grid_span * i / (n - 1) for i in range(n)]
        pts = []
        for j, fv in enumerate(fr):
            row = [(lo_u + fu * (hi_u - lo_u), lo_v + fv * (hi_v - lo_v)) for fu in fr]
            pts += row if j % 2 == 0 else row[::-1]          # snake order
        return pts

    def board_in_margin(pose):
        ci = node.cam_info
        _, uv, z = board_px(pose)
        mu, mv = a.edge_margin * ci['w'], a.edge_margin * ci['h']
        return (z > 0 and uv[:, 0].min() >= mu and uv[:, 0].max() <= ci['w'] - mu
                and uv[:, 1].min() >= mv and uv[:, 1].max() <= ci['h'] - mv)

    sign = {'pan': 1.0, 'tilt': 1.0}   # flipped online if the model is wrong

    def servo_to(target):
        """Move the head so the board centre lands on target (u, v)."""
        K = np.array(node.cam_info['K']).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        prev_err = None
        for it in range(a.servo_iters):
            t_rest = wait_still(False)
            d = latest_pose(t_rest)
            if d is None:
                return None, 'board lost'
            c, _, _ = board_px(d['pose'])
            eu, ev = target[0] - c[0], target[1] - c[1]
            err = math.hypot(eu, ev)
            if err <= a.servo_tol_px and board_in_margin(d['pose']):
                return d, None
            if prev_err is not None and err > 1.3 * prev_err:
                # model sign wrong on the dominant axis: flip it
                axis = 'pan' if abs(eu) >= abs(ev) else 'tilt'
                sign[axis] *= -1
                log(f'servo: {axis} sign flipped')
            prev_err = err
            # pinhole angle change; +pan (look left) moves the scene to +u,
            # +tilt (nose down) moves it to -v
            d_pan = sign['pan'] * (math.atan((target[0] - cx) / fx) - math.atan((c[0] - cx) / fx))
            d_tilt = -sign['tilt'] * (math.atan((target[1] - cy) / fy) - math.atan((c[1] - cy) / fy))
            s = node.snap()['js']
            tilt = s['head_joint1'] + a.servo_gain * d_tilt
            pan = s['head_joint2'] + a.servo_gain * d_pan
            m = a.joint_margin
            if not (HEAD_TILT_LIMITS[0] + m <= tilt <= HEAD_TILT_LIMITS[1] - m
                    and HEAD_PAN_LIMITS[0] + m <= pan <= HEAD_PAN_LIMITS[1] - m):
                return None, f'head limit (tilt {tilt:.3f}, pan {pan:.3f})'
            move_head(tilt, pan)
        return None, f'servo did not converge (err {prev_err:.0f} px)'

    def aim(pose, head0, target):
        """One head move that should put the board centre on target, predicted
        from the pose just captured (the board does not move while only the
        head moves). Returns None, or a skip reason."""
        K = np.array(node.cam_info['K']).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        c, _, _ = board_px(pose)
        d_pan = sign['pan'] * (math.atan((target[0] - cx) / fx) - math.atan((c[0] - cx) / fx))
        d_tilt = -sign['tilt'] * (math.atan((target[1] - cy) / fy) - math.atan((c[1] - cy) / fy))
        # relative to the head angles the reference pose was captured at, so a
        # failed capture in between does not leave the prediction stale
        tilt, pan = head0[0] + d_tilt, head0[1] + d_pan
        m = a.joint_margin
        if not (HEAD_TILT_LIMITS[0] + m <= tilt <= HEAD_TILT_LIMITS[1] - m
                and HEAD_PAN_LIMITS[0] + m <= pan <= HEAD_PAN_LIMITS[1] - m):
            return f'head limit (tilt {tilt:.3f}, pan {pan:.3f})'
        move_head(tilt, pan)
        return None

    def capture_sample():
        reason = 'no attempt'
        for attempt in range(2):
            try:
                t_rest = wait_still(use_base)
            except NotStill as e:
                reason = str(e)
                continue
            ps = passes_after(t_rest, a.passes, a.passes / 5.0 + 2.0)
            good = [d for d in ps if d.get('pose') and d['num_tags'] >= a.min_tags
                    and 0 <= d['reproj_px'] <= a.max_reproj and board_in_margin(d['pose'])]
            if len(good) < a.passes:
                reason = (f'{len(good)}/{a.passes} good passes in margin (tags '
                          f'{[d["num_tags"] for d in ps]}, reproj '
                          f'{[round(d["reproj_px"], 2) for d in ps]})')
                continue
            T = np.array([d['pose']['t'] for d in good])
            spread_mm = 1000 * np.max(np.linalg.norm(T - T.mean(0), axis=1))
            R0 = quat_to_R(*good[0]['pose']['q'])
            spread_deg = max(math.degrees(np.linalg.norm(R_to_rotvec(quat_to_R(*d['pose']['q']) @ R0.T)))
                             for d in good)
            if spread_mm > a.max_spread_mm or spread_deg > a.max_spread_deg:
                reason = f'unstable: {spread_mm:.2f} mm / {spread_deg:.2f} deg spread'
                continue
            return good, dict(spread_mm=spread_mm, spread_deg=spread_deg), None
        return None, None, reason

    # ── base motion (closed loop on /odom) ──────────────────────────────────
    def drive_to(start, dx, dy, dyaw, gain=1.2, tol_m=None, tol_deg=None):
        tol_m = a.base_tol_m if tol_m is None else tol_m
        tol_deg = a.base_tol_deg if tol_deg is None else tol_deg
        sx, sy, syaw = start['x'], start['y'], start['yaw']
        gx = sx + dx * math.cos(syaw) - dy * math.sin(syaw)
        gy = sy + dx * math.sin(syaw) + dy * math.cos(syaw)
        gyaw = syaw + dyaw
        o0 = node.snap()['odom']
        # guard: never get more than 10 cm further from the goal than at the start
        limit = math.hypot(gx - o0['x'], gy - o0['y']) + 0.10
        t0 = time.monotonic()
        node.publishing = True
        try:
            while True:
                s = node.snap()
                if s['odom_age'] is None or s['odom_age'] > 1.0:
                    # 1 s = the swerve controller's own cmd_vel_timeout; shorter gaps
                    # are this process stalling, not odom (bag showed none > 0.06 s)
                    raise RuntimeError(f'odom stale during base motion ({s["odom_age"]} s)')
                if s['odom_age'] > 0.3:
                    log(f'  (odom {s["odom_age"]:.2f} s old -- capture process stalled)')
                o = s['odom']
                if math.hypot(gx - o['x'], gy - o['y']) > limit:
                    raise RuntimeError('base overshoot guard tripped')
                ex_w, ey_w = gx - o['x'], gy - o['y']
                eyaw = math.atan2(math.sin(gyaw - o['yaw']), math.cos(gyaw - o['yaw']))
                if math.hypot(ex_w, ey_w) < tol_m and abs(eyaw) < math.radians(tol_deg):
                    break
                if time.monotonic() - t0 > 40:
                    raise RuntimeError('base move timed out')
                c, s_ = math.cos(o['yaw']), math.sin(o['yaw'])
                ex_b, ey_b = c * ex_w + s_ * ey_w, -s_ * ex_w + c * ey_w
                # ONE scale for translation and rotation: the twist keeps the
                # error's x:y:yaw ratio, so both finish together and every swerve
                # module's steering angle (set by that ratio) stays put. Separate
                # P loops drift the ratio, the steering targets swing, and the
                # controller zeroes all wheels until every module realigns
                # (0.57 deg when slow) -- a stop/steer/go tail on every step.
                k = min(gain, a.base_vmax / max(math.hypot(ex_b, ey_b), 1e-9),
                        a.base_wmax / max(abs(eyaw), 1e-9))
                tw = Twist()
                tw.linear.x, tw.linear.y = float(k * ex_b), float(k * ey_b)
                tw.angular.z = float(k * eyaw)
                node.cmd_pub.publish(tw)
                time.sleep(0.05)
        finally:
            stop_base()
            node.publishing = False

    # ── plan ────────────────────────────────────────────────────────────────
    lifts = [float(x) for x in a.lift.split(',')] if a.lift else []
    for z in lifts:
        if not (LIFT_LIMITS[0] <= z <= LIFT_LIMITS[1]):
            bail(f'lift {z} outside {LIFT_LIMITS}')
    if a.no_base:
        placements = [(0.0, 0.0, 0.0)]
    else:
        d, r = a.base_xy, math.radians(a.base_yaw_deg)
        placements = [(0, 0, 0), (d, 0, 0), (-d, 0, 0), (0, d, 0), (0, -d, 0), (0, 0, r), (0, 0, -r)]
    group_ids = list(range(len(placements)))
    if a.placements and not a.no_base:
        group_ids = [int(x) for x in a.placements.split(',')]
        placements = [placements[k] for k in group_ids]

    log('waiting for topics ...')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10 and not (node.cam_info and node.snap()['js']):
        time.sleep(0.1)
    if not node.cam_info:
        bail(f'no {a.ns}/camera_info -- is the head camera streaming?')
    s0 = node.snap()
    use_base = not a.no_base
    if use_base:
        if not a.plan_only:
            for _ in range(5):
                node.base_claim_pub.publish(Bool(data=True))
                time.sleep(0.02)
            node.base_claimed = True
            time.sleep(0.3)
            node.cmd_vel_others = 0              # count only what is still publishing
        time.sleep(1.0)
        s0 = node.snap()
        if not s0['odom'] or s0['odom_age'] > 0.5:
            bail('no live /odom (swerve_drive_controller active?) -- use --no-base')
        if node.cmd_vel_others and a.plan_only:
            log(f'note: {node.cmd_vel_others} /cmd_vel msgs/s from another node (a real run silences '
                'joy_base via /logitech/base_active)')
        elif node.cmd_vel_others:
            bail(f'{node.cmd_vel_others} /cmd_vel msgs from another node in the last '
                     'second -- stop that teleop first (or --no-base); joy_base is silenced '
                     'via /logitech/base_active, so this is something else')
    d0 = latest_pose(0, timeout=3.0)
    if d0 is None:
        bail(f'no board pose on {a.ns}/apriltag_detections -- board in view?')
    lifts = lifts or [s0['js'].get('lift_joint', 0.0)]
    n_targets = a.grid * a.grid
    total = len(placements) * len(lifts) * n_targets
    if a.grid_mode:
        log(f'plan: {len(placements)} base placements {placements if use_base else "(base off)"}')
        log(f'      lift heights {lifts}; {a.grid}x{a.grid} board-centre targets; '
            f'{total} samples max (~{total * 1.5 / 60:.0f} min)')
    else:
        log(f'plan: walk {a.steps} steps; base box x {a.base_x_range} y {a.base_y_range} m '
            f'+-{a.base_yaw_deg:.0f} deg '
            f'{"" if use_base else "(base off) "}step <= {a.walk_xy:.2f} m / {a.walk_yaw_deg:.0f} deg; '
            f'lift {a.walk_lift_range} step <= {a.walk_lift:.2f}; ~{a.steps * 1.3 / 60:.0f} min')
    log(f'      start: head tilt {s0["js"].get("head_joint1"):.3f} pan '
        f'{s0["js"].get("head_joint2"):.3f}, lift {s0["js"].get("lift_joint"):.3f}, '
        f'board {d0["num_tags"]} tags at {np.linalg.norm(d0["pose"]["t"]):.2f} m')
    if a.plan_only:
        shutdown()
        return
    if not a.yes:
        input('[calib] clear space around the base, arms clear of the lift travel -- '
              'Enter to start, Ctrl-C to abort: ')

    mount, err = None, None
    for _ in range(50):
        mount, err = node.lookup('camera_calibration_link', 'head_camera_frame')
        if mount:
            break
        time.sleep(0.1)
    if not mount:
        log(f'WARNING: current mount not in TF ({err}); solver will start from identity')
    out = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    f = open(out, 'a')

    def write(obj):
        f.write(json.dumps(obj) + '\n')
        f.flush()

    write(dict(type='header', time=time.time(), ns=a.ns, camera_info=node.cam_info,
               mount_calib_to_head_camera=mount, mount_err=err, args=vars(a),
               start=dict(js=s0['js'], odom=s0['odom']), placements=placements, lifts=lifts))
    log(f'writing {out}')

    start_odom = s0['odom']
    start_head = (s0['js']['head_joint1'], s0['js']['head_joint2'])
    start_lift = s0['js'].get('lift_joint')
    n_ok = 0
    t_last = time.monotonic()

    def cam_state(det_pose):
        """(board centre in base_link, camera position in base_link) for a
        camera-frame board pose, via the current TF base_link->head_camera_frame."""
        tf, _ = node.lookup('base_link', 'head_camera_frame')
        if tf is None:
            return None, None
        T = T_from(quat_to_R(*tf['q']), tf['t'])
        p_cam = quat_to_R(*det_pose['q']) @ BOARD_CENTER + np.array(det_pose['t'])
        return (T @ np.append(p_cam, 1.0))[:3], T[:3, 3]

    planar = planar_T

    def board_in_base(det_pose):
        tf, _ = node.lookup('base_link', 'head_camera_frame')
        if tf is None:
            return None
        return (T_from(quat_to_R(*tf['q']), tf['t'])
                @ T_from(quat_to_R(*det_pose['q']), det_pose['t']))

    walk_state = {}   # 'cur' (odom-commanded, rel. start) / 'vis' (board-measured)

    def bearing(v):
        return math.atan2(v[1], v[0]), math.atan2(v[2], math.hypot(v[0], v[1]))

    def walk():
        """Medium random steps of base + lift + head together (~0.5 s), one
        still capture after each. The head re-aim compensates the predicted
        board-bearing change of the base/lift step, aims the board at the image
        centre and adds a random offset (walk_target: tilt range = pan range x H/W)."""
        nonlocal n_ok, t_last
        rng = np.random.default_rng(a.seed)
        lo, hi = (float(x) for x in a.walk_lift_range.split(','))
        K = np.array(node.cam_info['K']).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        cur = np.zeros(3)  # base dx, dy, dyaw relative to start
        lift = node.snap()['js']['lift_joint']
        lift = min(hi, max(lo, lift))
        d = latest_pose(wait_still(use_base))
        if d is None:
            raise RuntimeError('board not visible at walk start')
        pose = d['pose']
        byaw = math.radians(a.base_yaw_deg) if use_base else 0.0
        xr = [float(v) for v in a.base_x_range.split(',')] if use_base else [0.0, 0.0]
        yr = [float(v) for v in a.base_y_range.split(',')] if use_base else [0.0, 0.0]
        # Odometry yaw drifts (~14 deg over 300 steps on 2026-10-09), so the
        # box and the final return use the base pose MEASURED from the board:
        # vis = start base -> current base, from T_base0_board @ inv(T_base_board).
        T_b0_board = board_in_base(pose)
        vis = np.zeros(3)
        walk_state.update(cur=cur, vis=vis)
        for step in range(a.steps):
            js = node.snap()['js']
            head0 = np.array([js['head_joint1'], js['head_joint2']])
            p_b, cam_b = cam_state(pose)
            if p_b is None:
                raise RuntimeError('no TF base_link->head_camera_frame')
            # propose a medium step of everything
            nxt = cur.copy()
            if use_base:
                # step chosen so the MEASURED pose stays in the box, applied as
                # the same delta on the odom-commanded pose
                want = np.array([
                    np.clip(vis[0] + rng.uniform(-a.walk_xy, a.walk_xy), xr[0], xr[1]),
                    np.clip(vis[1] + rng.uniform(-a.walk_xy, a.walk_xy), yr[0], yr[1]),
                    np.clip(vis[2] + math.radians(rng.uniform(-a.walk_yaw_deg, a.walk_yaw_deg)),
                            -byaw, byaw)])
                nxt = cur + (want - vis)
            n_lift = float(np.clip(lift + rng.uniform(-a.walk_lift, a.walk_lift), lo, hi))
            # predicted bearing change of the board centre seen from the camera
            T_new_old = np.linalg.inv(planar(*nxt)) @ planar(*cur)
            p_new = (T_new_old @ np.append(p_b, 1.0))[:3]
            cam_new = cam_b + np.array([0, 0, n_lift - lift])
            az0, el0 = bearing(p_b - cam_b)
            az1, el1 = bearing(p_new - cam_new)
            c, _, _ = board_px(pose)
            goal = None
            for _try in range(6):
                tu, tv = walk_target(pose, rng)
                if _try == 5:
                    tu, tv = c[0], c[1]       # last resort: just keep the board where it is
                d_pan = (az1 - az0) + (math.atan((tu - cx) / fx) - math.atan((c[0] - cx) / fx))
                d_tilt = -(el1 - el0) - (math.atan((tv - cy) / fy) - math.atan((c[1] - cy) / fy))
                cand = head0 + np.array([d_tilt, d_pan])
                m = a.joint_margin
                if (HEAD_TILT_LIMITS[0] + m <= cand[0] <= HEAD_TILT_LIMITS[1] - m
                        and HEAD_PAN_LIMITS[0] + m <= cand[1] <= HEAD_PAN_LIMITS[1] - m):
                    goal = cand
                    break
            if goal is None:
                # head can't follow this base/lift step: undo it, keep head
                nxt, n_lift, goal = cur.copy(), lift, head0
            # move everything at once
            ev = {}                                     # wall-clock event times of this step

            def mark(k):
                ev[k] = time.time()
            t_mv = time.monotonic()
            dh = float(np.max(np.abs(goal - head0)))
            head_dur = max(a.head_min_s, dh / a.head_vmax)
            lift_dur = max(a.head_min_s, abs(n_lift - lift) / a.lift_vmax)
            mark('head_send')
            hh = send_traj(node.head_ac, ['head_joint1', 'head_joint2'], list(goal), head_dur, wait=False)
            mark('head_accepted')
            if rail.on:
                rail.set(n_lift)
                hl = None
            else:
                hl = send_traj(node.lift_ac, ['lift_joint'], [n_lift], lift_dur, wait=False)
            mark('lift_accepted')
            if use_base and np.any(nxt != cur):
                mark('base_start')
                drive_to(start_odom, nxt[0], nxt[1], nxt[2], gain=a.walk_gain,
                         tol_m=a.walk_tol_m, tol_deg=a.walk_tol_deg)
                mark('base_done')
            wait_traj(hh)
            mark('head_result')
            if hl is not None:
                wait_traj(hl)
            elif not rail.wait():
                raise RuntimeError(f'lift did not follow the rail to {n_lift:+.3f} '
                                   f'(at {node.snap()["js"].get("lift_joint")})')
            mark('lift_result')
            t_cap = time.monotonic()
            cur, lift = nxt, n_lift
            good, stab, why = capture_sample()
            mark('capture_done')
            write(dict(type='event', mode='walk', step=step, events=ev,
                       head_goal=[float(g) for g in goal], head_start=[float(h) for h in head0], head_dur=head_dur,
                       lift_goal=n_lift, lift_dur=lift_dur, base_goal=nxt.tolist(), ok=good is not None))
            t_done = time.monotonic()
            if good is None:
                # likely drifted out of the margin: re-centre from a fresh pose, retry once
                try:
                    d = latest_pose(wait_still(use_base))
                except NotStill as e:
                    d, why = None, str(e)
                if d is not None:
                    js = node.snap()['js']
                    if aim(d['pose'], (js['head_joint1'], js['head_joint2']), (cx, cy)) is None:
                        good, stab, why = capture_sample()
                if good is None:
                    log(f'  walk {step}: skip ({why})')
                    write(dict(type='skip', mode='walk', step=step, reason=why))
                    if d is not None:
                        pose = d['pose']
                    continue
            snap = node.snap()
            calib, terr = node.lookup('base_link', 'camera_calibration_link')
            write(dict(type='sample', mode='walk', time=time.time(), group='walk', step=step,
                       base_rel=cur.tolist(), lift_target=lift, target_px=[0, 0],
                       odom=snap['odom'], joints=snap['js'], head_camera_tf=snap['cam_tf'],
                       base_to_calib=calib, tf_err=terr, stability=stab, passes=good,
                       imu=imu_record(), head_links=head_links()))
            n_ok += 1
            pose = good[-1]['pose']
            Tbi = board_in_base(pose)
            if Tbi is not None and T_b0_board is not None:
                V = T_b0_board @ np.linalg.inv(Tbi)
                vis = np.array([V[0, 3], V[1, 3], math.atan2(V[1, 0], V[0, 0])])
            walk_state.update(cur=cur, vis=vis)
            c, _, _ = board_px(pose)
            log(f'  walk {step}: sample {n_ok}  base ({vis[0]:+.3f},{vis[1]:+.3f},'
                f'{math.degrees(vis[2]):+.1f}deg; odom yaw {math.degrees(cur[2]):+.1f}) '
                f'lift {lift:+.3f}  centre ({c[0]:.0f},{c[1]:.0f})  '
                f'move {t_cap - t_mv:.2f}s capture {t_done - t_cap:.2f}s'
                f'  reproj {good[-1]["reproj_px"]:.2f}px  [{time.monotonic() - t_last:.1f}s]')
            t_last = time.monotonic()

    try:
      rail.start()
      if not a.grid_mode:
        walk()
      else:
        for g, (dx, dy, dyaw) in zip(group_ids, placements):
            if use_base:
                log(f'base -> placement {g}: dx {dx:+.2f} dy {dy:+.2f} dyaw {math.degrees(dyaw):+.1f}')
                drive_to(start_odom, dx, dy, dyaw)
            for li, z in enumerate(lifts if group_ids.index(g) % 2 == 0 else lifts[::-1]):
                log(f'lift -> {z:+.3f}')
                move_lift(z)
                d = latest_pose(wait_still(use_base))
                if d is None:
                    log('board not visible after lift move -- head back to start pose')
                    move_head(*start_head)
                    d = latest_pose(wait_still(use_base))
                    if d is None:
                        write(dict(type='skip', group=g, lift=z, reason='board not visible'))
                        continue
                pose = d['pose']
                js = node.snap()['js']
                head0 = (js['head_joint1'], js['head_joint2'])
                for ti, tgt in enumerate(targets_for(d['pose'])):
                    if a.servo:
                        det, why = servo_to(tgt)
                    else:
                        det, why = True, aim(pose, head0, tgt)
                    if det is None or why:
                        log(f'  g{g} lift {z:+.2f} target {ti}: skip ({why})')
                        write(dict(type='skip', group=g, lift=z, target=tgt, reason=why))
                        continue
                    good, stab, why = capture_sample()
                    if good is None:
                        log(f'  g{g} lift {z:+.2f} target {ti}: skip ({why})')
                        write(dict(type='skip', group=g, lift=z, target=tgt, reason=why))
                        continue
                    snap = node.snap()
                    calib, terr = node.lookup('base_link', 'camera_calibration_link')
                    write(dict(type='sample', time=time.time(), group=g,
                               placement=[dx, dy, dyaw], lift_target=z, target_px=tgt,
                               odom=snap['odom'], joints=snap['js'], head_camera_tf=snap['cam_tf'],
                               base_to_calib=calib, tf_err=terr, stability=stab, passes=good,
                               imu=imu_record(), head_links=head_links()))
                    n_ok += 1
                    pose = good[-1]['pose']
                    head0 = (snap['js']['head_joint1'], snap['js']['head_joint2'])
                    c, _, _ = board_px(good[-1]['pose'])
                    log(f'  g{g} lift {z:+.2f} target {ti}: sample {n_ok}  centre '
                        f'({c[0]:.0f},{c[1]:.0f})  tags {good[-1]["num_tags"]}  reproj '
                        f'{good[-1]["reproj_px"]:.2f}px  spread {stab["spread_mm"]:.2f}mm  '
                        f'[{time.monotonic() - t_last:.1f}s]')
                    t_last = time.monotonic()
    except KeyboardInterrupt:
        log('interrupted')
    finally:
        stop_base()
        log(f'{n_ok} samples written to {out} ({len(node.motion_drops)} detection passes dropped '
            f'for camera motion); returning to start pose')
        try:
            move_head(*start_head)
            if start_lift is not None:
                move_lift(start_lift)
            if use_base:
                if 'vis' in walk_state:
                    # odom pose that the board says is the true start
                    back = walk_state['cur'] - walk_state['vis']
                    log(f'return: odom drift corrected by ({back[0]:+.3f}, {back[1]:+.3f}, '
                        f'{math.degrees(back[2]):+.1f} deg)')
                    drive_to(start_odom, *back)
                else:
                    drive_to(start_odom, 0.0, 0.0, 0.0)
        except Exception as e:  # noqa: BLE001
            log(f'return to start failed: {e}')
        stop_base()
        try:
            rail.stop()
        except Exception as e:  # noqa: BLE001
            log(f'lift rail release failed: {e} -- publish "ee" to /teleop_goal_source')
        f.close()
        shutdown()


# ══════════════════════════════════════════════════════════════════════ SOLVE
def solve(paths, args):
    from scipy.optimize import least_squares

    # Several capture files may be combined: a placement's board pose is only
    # shared within one file (each run starts from its own start pose), so the
    # group key is (file, placement).
    header, samples = None, []
    for fi, path in enumerate(paths):
        with open(os.path.expanduser(path)) as fh:
            for line in fh:
                r = json.loads(line)
                if r['type'] == 'header' and header is None:
                    header = r
                elif r['type'] == 'sample' and r.get('base_to_calib'):
                    r['group'] = f'{fi}:{r["group"]}'
                    samples.append(r)
    path = ', '.join(paths)
    if not samples:
        sys.exit('no samples with base_to_calib in ' + path)
    ci = header['camera_info']
    K = np.array(ci['K']).reshape(3, 3)
    Dist = np.array(ci['D'] or [0.0] * 5)
    if np.any(np.abs(Dist) > 1e-9):
        print('note: non-zero distortion ignored by this solver (D435 RGB is ~undistorted)')

    groups = sorted({s['group'] for s in samples})
    gidx = {g: i for i, g in enumerate(groups)}
    N, G_ = len(samples), len(groups)
    obs = []  # (sample i, group, 3D board pt, observed uv)
    for i, s in enumerate(samples):
        for p in s['passes']:
            for tag in p['tags']:
                tid = int(tag[0])
                if tid not in BOARD:
                    continue
                uv = np.array(tag[2:10]).reshape(4, 2)
                for k in range(4):
                    obs.append((i, gidx[s['group']], BOARD[tid][k], uv[k]))
    si = np.array([o[0] for o in obs])
    idx_of = [np.flatnonzero(si == i) for i in range(N)]
    gi = np.array([o[1] for o in obs])
    Xb = np.array([o[2] for o in obs])
    UV = np.array([o[3] for o in obs])
    Xh = np.hstack([Xb, np.ones((len(Xb), 1))])
    n_obs = 2 * len(obs)

    # Per sample: Abc = base_link -> camera_calibration_link (TF, joint-predicted),
    # Od = planar odom pose for walk samples (the board is fixed in odom there), else I.
    Abc = np.array([T_from(quat_to_R(*s['base_to_calib']['q']), s['base_to_calib']['t']) for s in samples])
    Od = np.array([planar_T(s['odom']['x'], s['odom']['y'], s['odom']['yaw']) if s.get('mode') == 'walk'
                   else np.eye(4) for s in samples])
    walk_idx = [i for i, s in enumerate(samples) if s.get('mode') == 'walk']
    walk_arr = np.array(walk_idx, dtype=int)
    g_of_sample = np.array([gidx[s['group']] for s in samples])

    # Head play (backlash): samples that recorded base_link -> head_link1/2 get a small
    # rotation about each head joint's axis through its pivot (tilt: head_link1 y, pan:
    # head_link2 z), soft prior --play-sigma-deg. The board fixes the camera's full pose
    # per sample, so the play is observable; without it the play leaks into the mount.
    play_idx = [i for i, s in enumerate(samples)
                if not args.no_play and s.get('head_links') and all(s['head_links'].get(k) for k in ('head_link1', 'head_link2'))]
    play_arr = np.array(play_idx, dtype=int)
    if play_idx:
        def link(i, k):
            h = samples[i]['head_links'][k]
            return quat_to_R(*h['q']), np.array(h['t'])
        P1 = np.array([link(i, 'head_link1')[1] for i in play_idx])
        A1 = np.array([link(i, 'head_link1')[0][:, 1] for i in play_idx])
        P2 = np.array([link(i, 'head_link2')[1] for i in play_idx])
        A2 = np.array([link(i, 'head_link2')[0][:, 2] for i in play_idx])

    # Gravity: samples with a still IMU window. Measured up (accel direction) is in the
    # D435i's d435_camera frame == head_camera_frame (factory IMU->color extrinsics);
    # an IMU->camera rotation correction Ric (3, prior --ic-sigma-deg) and a floor tilt
    # (2, prior --floor-sigma-deg; up in the samples' frame: odom for walk, base_link
    # otherwise) are solved with it.
    grav_idx = [i for i, s in enumerate(samples)
                if not args.no_imu and s.get('imu') and not s['imu'].get('moving')]
    grav_arr = np.array(grav_idx, dtype=int)
    use_imu = len(grav_idx) >= 3
    if use_imu:
        U_meas = np.array([np.array(samples[i]['imu']['accel']) / np.linalg.norm(samples[i]['imu']['accel'])
                           for i in grav_idx])
        g_norm = np.array([np.linalg.norm(samples[i]['imu']['accel']) for i in grav_idx])

    m0 = header.get('mount_calib_to_head_camera')
    M0 = T_from(quat_to_R(*m0['q']), m0['t']) if m0 else np.eye(4)

    def T_cam_board(p):
        return T_from(quat_to_R(*p['pose']['q']), p['pose']['t'])

    # parameter layout
    o_B = 6
    nb = 6 + 6 * G_
    o_ic = nb
    o_fl = nb + 3
    n_glob = nb + (5 if use_imu else 0)
    o_pl = n_glob
    o_wc = o_pl + 2 * len(play_idx)
    n_x = o_wc + 3 * len(walk_idx)
    play_pos = {i: k for k, i in enumerate(play_idx)}
    walk_pos = {i: k for k, i in enumerate(walk_idx)}

    # initial values: mount from the recorded TF, boards from the median detection,
    # walk corrections anchored at each run's first walk sample (as before)
    x0 = np.zeros(n_x)
    x0[0:3], x0[3:6] = M0[:3, 3], R_to_rotvec(M0[:3, :3])
    A0 = Od @ Abc
    for g in groups:
        Ts = [A0[i] @ M0 @ T_cam_board(s['passes'][0]) for i, s in enumerate(samples) if s['group'] == g]
        k = gidx[g]
        x0[o_B + 6 * k:o_B + 6 * k + 3] = np.median([T[:3, 3] for T in Ts], axis=0)
        x0[o_B + 6 * k + 3:o_B + 6 * k + 6] = R_to_rotvec(Ts[0][:3, :3])
    anchor = {}
    for k, i in enumerate(walk_idx):
        T_base_board = Abc[i] @ M0 @ T_cam_board(samples[i]['passes'][0])
        g = samples[i]['group']
        if g not in anchor:
            anchor[g] = Od[i] @ T_base_board            # board in odom
        C = np.linalg.inv(Od[i]) @ anchor[g] @ np.linalg.inv(T_base_board)
        x0[o_wc + 3 * k:o_wc + 3 * k + 3] = [C[0, 3], C[1, 3], math.atan2(C[1, 0], C[0, 0])]
    for g, Tb0 in anchor.items():
        k = gidx[g]
        x0[o_B + 6 * k:o_B + 6 * k + 3] = Tb0[:3, 3]
        x0[o_B + 6 * k + 3:o_B + 6 * k + 6] = R_to_rotvec(Tb0[:3, :3])

    def rot_batch(ax, th):
        """Rodrigues for N unit axes (N,3) and angles (N,) -> (N,3,3)."""
        K = np.zeros((len(ax), 3, 3))
        K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -ax[:, 2], ax[:, 1], -ax[:, 0]
        K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = ax[:, 2], -ax[:, 1], ax[:, 0]
        s_, c_ = np.sin(th)[:, None, None], np.cos(th)[:, None, None]
        return np.eye(3)[None] + s_ * K + (1 - c_) * (K @ K)

    def rot_at(p, ax, th):
        """(N,4,4): rotation by th about the axis ax through point p."""
        T = np.zeros((len(ax), 4, 4))
        R = rot_batch(ax, th)
        T[:, :3, :3] = R
        T[:, :3, 3] = p - np.einsum('nij,nj->ni', R, p)
        T[:, 3, 3] = 1.0
        return T

    def planar_batch(c):
        cy, sy = np.cos(c[:, 2]), np.sin(c[:, 2])
        T = np.zeros((len(c), 4, 4))
        T[:, 0, 0], T[:, 0, 1], T[:, 1, 0], T[:, 1, 1] = cy, -sy, sy, cy
        T[:, 0, 3], T[:, 1, 3] = c[:, 0], c[:, 1]
        T[:, 2, 2] = T[:, 3, 3] = 1.0
        return T

    def cam_poses(x):
        """(N,4,4) camera_calibration_link in each sample's group frame, and M."""
        M = T_from(rotvec_to_R(x[3:6]), x[0:3])
        Ab = Abc
        if play_idx:
            d = x[o_pl:o_wc].reshape(-1, 2)
            Ab = Abc.copy()
            Ab[play_arr] = rot_at(P1, A1, d[:, 0]) @ rot_at(P2, A2, d[:, 1]) @ Abc[play_arr]
        Odx = Od
        if walk_idx:
            Odx = Od.copy()
            Odx[walk_arr] = Od[walk_arr] @ planar_batch(x[o_wc:].reshape(-1, 3))
        return Odx @ Ab, M

    sig_g = math.radians(args.grav_sigma_deg)

    def residual(x):
        A, M = cam_poses(x)
        Tb = np.array([T_from(rotvec_to_R(x[o_B + 6 * k + 3:o_B + 6 * k + 6]), x[o_B + 6 * k:o_B + 6 * k + 3])
                       for k in range(G_)])
        AM = A @ M
        T = np.linalg.inv(AM) @ Tb[g_of_sample]                  # (N,4,4) T_cam_board
        P = np.einsum('oij,oj->oi', T[si, :3, :], Xh)
        uv = (P[:, :2] / P[:, 2:3]) * K[[0, 1], [0, 1]] + K[[0, 1], [2, 2]]
        out = [(uv - UV).ravel()]
        if walk_idx:
            out.append(x[o_wc:].reshape(-1, 3)[:, :2].ravel() / args.odom_sigma_m)
        if play_idx:
            out.append(x[o_pl:o_wc] / math.radians(args.play_sigma_deg))
        if use_imu:
            out.append(x[o_ic:o_ic + 3] / math.radians(args.ic_sigma_deg))
            out.append(x[o_fl:o_fl + 2] / math.radians(args.floor_sigma_deg))
            up = rotvec_to_R([x[o_fl], x[o_fl + 1], 0.0]) @ np.array([0.0, 0.0, 1.0])
            Rc = AM[grav_arr, :3, :3]
            pred = np.einsum('nji,j->ni', Rc, up)                    # up in the camera
            meas = U_meas @ rotvec_to_R(x[o_ic:o_ic + 3]).T
            out.append((pred - meas).ravel() / sig_g)
        return np.concatenate(out)

    # Jacobian sparsity
    from scipy.sparse import lil_matrix
    n_res = residual(x0).size
    sp = lil_matrix((n_res, n_x), dtype=int)
    for i in range(N):
        rows = np.concatenate([2 * idx_of[i], 2 * idx_of[i] + 1])
        sp[rows, 0:6] = 1
        k = g_of_sample[i]
        sp[rows, o_B + 6 * k:o_B + 6 * k + 6] = 1
        if i in play_pos:
            sp[rows, o_pl + 2 * play_pos[i]:o_pl + 2 * play_pos[i] + 2] = 1
        if i in walk_pos:
            sp[rows, o_wc + 3 * walk_pos[i]:o_wc + 3 * walk_pos[i] + 3] = 1
    r_ = n_obs
    for k in range(len(walk_idx)):
        sp[r_ + 2 * k, o_wc + 3 * k] = 1
        sp[r_ + 2 * k + 1, o_wc + 3 * k + 1] = 1
    r_ += 2 * len(walk_idx)
    for k in range(2 * len(play_idx)):
        sp[r_ + k, o_pl + k] = 1
    r_ += 2 * len(play_idx)
    if use_imu:
        for k in range(5):
            sp[r_ + k, o_ic + k] = 1
        r_ += 5
        for k, i in enumerate(grav_idx):
            rows = np.arange(r_ + 3 * k, r_ + 3 * k + 3)
            sp[rows, 3:6] = 1
            sp[rows, o_ic:o_ic + 5] = 1
            if i in play_pos:
                sp[rows, o_pl + 2 * play_pos[i]:o_pl + 2 * play_pos[i] + 2] = 1
            if i in walk_pos:
                sp[rows, o_wc + 3 * walk_pos[i]:o_wc + 3 * walk_pos[i] + 3] = 1

    rr0 = residual(x0)
    r0 = rr0[:n_obs]
    # tolerances: 1e-6 relative is ~0.01 mm / 0.001 deg on the mount
    res = least_squares(residual, x0, loss='soft_l1', f_scale=1.0, max_nfev=300,
                        jac_sparsity=sp, x_scale='jac', ftol=args.tol, xtol=args.tol, gtol=args.tol)
    xs = res.x
    J = res.jac
    s2 = float(np.sum(res.fun[:n_obs] ** 2) / max(1, n_obs - n_x))
    try:
        JTJ = J.T @ J
        JTJ = JTJ.toarray() if hasattr(JTJ, 'toarray') else JTJ
        cov = np.linalg.pinv(JTJ) * s2
        sd_all = np.sqrt(np.abs(np.diag(cov)))
    except np.linalg.LinAlgError:
        sd_all = np.full(n_x, np.nan)
    sd = sd_all[:6]
    if walk_idx:
        wcs = xs[o_wc:].reshape(-1, 3)
        print(f'walk base corrections vs odom: |xy| median {1000 * np.median(np.hypot(wcs[:, 0], wcs[:, 1])):.0f} mm, '
              f'yaw range {math.degrees(wcs[:, 2].min()):+.1f}..{math.degrees(wcs[:, 2].max()):+.1f} deg')
    if play_idx:
        d = np.degrees(xs[o_pl:o_wc].reshape(-1, 2))
        print(f'head play over {len(play_idx)} samples (prior {args.play_sigma_deg} deg): '
              f'tilt {d[:, 0].min():+.3f}..{d[:, 0].max():+.3f} (rms {np.sqrt(np.mean(d[:, 0] ** 2)):.3f}), '
              f'pan {d[:, 1].min():+.3f}..{d[:, 1].max():+.3f} (rms {np.sqrt(np.mean(d[:, 1] ** 2)):.3f}) deg')
    else:
        print('head play: not modelled (no head_link1/2 in the samples, or --no-play)')
    imu_out = None
    if use_imu:
        ic = xs[o_ic:o_ic + 3]
        fl = xs[o_fl:o_fl + 2]
        rg = res.fun[-3 * len(grav_idx):].reshape(-1, 3) * sig_g
        rg0 = rr0[-3 * len(grav_idx):].reshape(-1, 3) * sig_g
        ang = lambda r: np.degrees(np.linalg.norm(r, axis=1))
        print(f'IMU: {len(grav_idx)} gravity samples, |a| {g_norm.min():.3f}..{g_norm.max():.3f} m/s^2; '
              f'gravity residual {np.sqrt(np.mean(ang(rg0) ** 2)):.3f} -> {np.sqrt(np.mean(ang(rg) ** 2)):.3f} deg rms '
              f'(max {ang(rg).max():.3f})')
        print(f'  IMU -> camera correction rotvec {np.degrees(ic).round(3)} deg '
              f'(|{math.degrees(np.linalg.norm(ic)):.3f}|, 1-sigma {np.degrees(sd_all[o_ic:o_ic + 3]).round(3)})')
        print(f'  floor tilt (up in the sample frame) rotvec x {math.degrees(fl[0]):+.3f} y {math.degrees(fl[1]):+.3f} deg '
              f'(1-sigma {np.degrees(sd_all[o_fl:o_fl + 2]).round(3)})')
        imu_out = dict(imu_cam_rotvec=ic.tolist(), floor_up_rotvec=[float(fl[0]), float(fl[1]), 0.0],
                       imu_cam_sigma=sd_all[o_ic:o_ic + 3].tolist(), floor_sigma=sd_all[o_fl:o_fl + 2].tolist(),
                       n_gravity=len(grav_idx),
                       gravity_rms_deg=float(np.sqrt(np.mean(ang(rg) ** 2))))
    else:
        print('IMU: not used (fewer than 3 still IMU samples, or --no-imu)')
    res.fun = res.fun[:n_obs]
    res.x = xs

    r = res.fun.reshape(-1, 2)
    px = np.linalg.norm(r, axis=1)
    M = T_from(rotvec_to_R(res.x[3:6]), res.x[0:3])
    # static_transform_publisher roll/pitch/yaw: R = Rz(yaw) Ry(pitch) Rx(roll)
    R = M[:3, :3]
    pitch = math.asin(-max(-1, min(1, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])

    print(f'samples {len(samples)}  groups {len(groups)}  corners {len(obs)}')
    print(f'reprojection RMS: start {np.sqrt(np.mean(np.linalg.norm(r0.reshape(-1, 2), axis=1) ** 2)):.2f} px'
          f' -> solved {np.sqrt(np.mean(px ** 2)):.2f} px  (median {np.median(px):.2f}, '
          f'95% {np.percentile(px, 95):.2f})')
    print('mount camera_calibration_link -> head_camera_frame:')
    print(f'  t = ({M[0, 3]:+.4f}, {M[1, 3]:+.4f}, {M[2, 3]:+.4f}) m   '
          f'1-sigma ({sd[0] * 1000:.2f}, {sd[1] * 1000:.2f}, {sd[2] * 1000:.2f}) mm')
    print(f'  rotvec 1-sigma ({math.degrees(sd[3]):.3f}, {math.degrees(sd[4]):.3f}, '
          f'{math.degrees(sd[5]):.3f}) deg')
    print('  static_transform_publisher args:')
    print(f"   '--x', '{M[0, 3]:.6f}', '--y', '{M[1, 3]:.6f}', '--z', '{M[2, 3]:.6f}',")
    print(f"   '--roll', '{roll:.6f}', '--pitch', '{pitch:.6f}', '--yaw', '{yaw:.6f}'")
    if m0:
        D = np.linalg.inv(M0) @ M
        print(f'  change vs recorded mount: {np.linalg.norm(D[:3, 3]) * 1000:.1f} mm, '
              f'{math.degrees(np.linalg.norm(R_to_rotvec(D[:3, :3]))):.2f} deg')
    for g in groups:
        sel = gi == gidx[g]
        n_s = sum(1 for s in samples if s['group'] == g)
        print(f'  group {g}: {n_s} samples, {int(sel.sum())} corners, '
              f'RMS {np.sqrt(np.mean(px[sel] ** 2)):.2f} px')
    per = [(np.sqrt(np.mean(px[ix] ** 2)), i) for i, ix in enumerate(idx_of) if ix.size]
    per.sort(reverse=True)
    print('worst samples (RMS px, index, group, lift, target):')
    for v, i in per[:5]:
        s = samples[i]
        print(f'  {v:6.2f}  #{i}  g{s["group"]}  lift {s["lift_target"]:+.2f}  '
              f'({s["target_px"][0]:.0f},{s["target_px"][1]:.0f})')
    out_path = os.path.expanduser(args.solve_out) if args.solve_out else         os.path.splitext(os.path.expanduser(paths[0]))[0] + '_solve.json'
    with open(out_path, 'w') as fh:
        json.dump(dict(inputs=list(paths), time=time.time(), mount=dict(
            t=M[:3, 3].tolist(), roll=roll, pitch=pitch, yaw=yaw,
            sigma_mm=(sd[:3] * 1000).tolist(), sigma_deg=np.degrees(sd[3:6]).tolist()),
            reproj_rms_px=float(np.sqrt(np.mean(px ** 2))), imu=imu_out,
            play_deg=(np.degrees(res.x[o_pl:o_wc]).reshape(-1, 2).tolist() if play_idx else None),
            play_sample_index=play_idx), fh, indent=1)
    print(f'saved {out_path}' + ('' if imu_out is None else
          '  (head_camera_tf_bridge imu_calib_file: the runtime IMU tilt correction reads it)'))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--tol', type=float, default=1e-6, help='solve: ftol/xtol/gtol')
    ap.add_argument('--odom-sigma-m', type=float, default=0.03,
                    help='solve: soft prior on walk samples\' x/y odometry (yaw is free)')
    ap.add_argument('--no-play', action='store_true', help='solve: no per-sample head play unknowns')
    ap.add_argument('--play-sigma-deg', type=float, default=0.5,
                    help='solve: prior on the per-sample tilt/pan play (2026-10-12: ~1 deg tilt, ~0.5 pan)')
    ap.add_argument('--no-imu', action='store_true', help='solve: ignore the IMU gravity')
    ap.add_argument('--grav-sigma-deg', type=float, default=0.05, help='solve: gravity direction noise')
    ap.add_argument('--ic-sigma-deg', type=float, default=1.0, help='solve: prior on the IMU->camera correction')
    ap.add_argument('--floor-sigma-deg', type=float, default=1.0, help='solve: prior on the floor tilt')
    ap.add_argument('--solve-out', default='', help='solve: result json (default <first input>_solve.json)')
    ap.add_argument('--solve', metavar='JSONL', nargs='+',
                    help='solve one or more capture files instead of capturing')
    ap.add_argument('--ns', default='/head_camera',
                    help='board-tap namespace (both head cameras publish /head_camera/*)')
    ap.add_argument('--out', default=f'~/head_cam_calib_{time.strftime("%Y%m%d_%H%M%S")}.jsonl')
    ap.add_argument('--plan-only', action='store_true')
    ap.add_argument('--yes', action='store_true', help='do not wait for Enter')
    ap.add_argument('--no-base', action='store_true')
    ap.add_argument('--grid-mode', action='store_true',
                    help='old structured mode: fixed placements x lifts x head grid')
    ap.add_argument('--steps', type=int, default=300, help='walk mode: number of steps')
    ap.add_argument('--base-x-range', default='-0.30,0.10',
                    help='walk: allowed base x relative to start [m] (board-measured), "lo,hi"')
    ap.add_argument('--base-y-range', default='-0.30,0.10',
                    help='walk: allowed base y relative to start [m] (board-measured), "lo,hi"')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--walk-xy', type=float, default=0.05, help='max base x/y step [m]')
    ap.add_argument('--walk-yaw-deg', type=float, default=3.0, help='max base yaw step')
    ap.add_argument('--walk-lift', type=float, default=0.04, help='max lift step [m]')
    ap.add_argument('--walk-lift-range', default='-0.26,-0.06', help='lift range for the walk')
    ap.add_argument('--walk-gain', type=float, default=4.0, help='base P gain for walk steps')
    ap.add_argument('--walk-tol-m', type=float, default=0.03,
                    help='walk steps only need to get close -- odom records where the base stopped')
    ap.add_argument('--walk-tol-deg', type=float, default=1.5)
    ap.add_argument('--base-xy', type=float, default=0.20, help='+-x and +-y placement [m]')
    ap.add_argument('--placements', default='',
                    help='subset of placement indices to run, e.g. "2,3,4,5,6" '
                         '(0 start, 1 +x, 2 -x, 3 +y, 4 -y, 5 +yaw, 6 -yaw)')
    ap.add_argument('--base-yaw-deg', type=float, default=10.0)
    ap.add_argument('--base-vmax', type=float, default=0.12)
    ap.add_argument('--base-wmax', type=float, default=0.20)
    ap.add_argument('--base-tol-m', type=float, default=0.01)
    ap.add_argument('--base-tol-deg', type=float, default=0.5)
    ap.add_argument('--lift', default='-0.10,-0.18,-0.26',
                    help='lift heights, comma list ("" = stay); 0 = top, min -0.5')
    ap.add_argument('--lift-vmax', type=float, default=0.10)
    ap.add_argument('--head-vmax', type=float, default=0.8)
    ap.add_argument('--head-min-s', type=float, default=0.25, help='shortest head move [s]')
    ap.add_argument('--grid', type=int, default=5, help='NxN board-centre image targets')
    ap.add_argument('--walk-aim-frac', type=float, default=0.6,
                    help='walk: random board-centre offset around the principal point, as a fraction '
                         'of the room before the board leaves --edge-margin (tilt range = pan range x H/W)')
    ap.add_argument('--grid-span', type=float, default=0.6,
                    help='fraction of the board-fits-in-margin range the grid spans (1 = edge to edge)')
    ap.add_argument('--edge-margin', type=float, default=0.08,
                    help='whole board kept this fraction of W/H away from the edges')
    ap.add_argument('--servo', action='store_true',
                    help='closed-loop head servo per target (slow); default is one predicted move')
    ap.add_argument('--servo-iters', type=int, default=4)
    ap.add_argument('--servo-tol-px', type=float, default=25.0)
    ap.add_argument('--servo-gain', type=float, default=0.9)
    ap.add_argument('--joint-margin', type=float, default=0.03, help='rad off head limits')
    ap.add_argument('--still-s', type=float, default=0.15)
    ap.add_argument('--imu-ns', default='/d435', help='D435i IMU topics: <ns>/{accel,gyro}/sample')
    ap.add_argument('--imu-still-s', type=float, default=0.5,
                    help='capture: gyro window that must be still (head play settles) [s]')
    ap.add_argument('--imu-still-dps', type=float, default=0.3,
                    help='capture: |mean gyro| over --imu-still-s below [deg/s] (residual bias ~0.14 at rest)')
    ap.add_argument('--imu-peak-dps', type=float, default=5.0,
                    help='capture: gyro peak over --imu-still-s below [deg/s] (noise spikes reach ~3.4 at rest)')
    ap.add_argument('--det-still-dps', type=float, default=1.0,
                    help='capture: drop a detection pass whose frame saw a gyro peak above this '
                         '[deg/s] (1 deg/s ~ 0.5 px blur at 30 ms exposure)')
    ap.add_argument('--det-motion-pre-s', type=float, default=0.1,
                    help='capture: gyro window before the frame capture stamp (exposure + readout) [s]')
    ap.add_argument('--still-joint-vel', type=float, default=0.005)
    ap.add_argument('--passes', type=int, default=2)
    ap.add_argument('--min-tags', type=int, default=4,
                    help='tags per pass (the 15 mm tags are ~14 px at 1 m and rarely detect)')
    ap.add_argument('--max-reproj', type=float, default=1.5)
    ap.add_argument('--max-spread-mm', type=float, default=1.5)
    ap.add_argument('--max-spread-deg', type=float, default=0.2)
    a = ap.parse_args()
    if a.solve:
        solve(a.solve, a)
    else:
        capture(a)


if __name__ == '__main__':
    main()
