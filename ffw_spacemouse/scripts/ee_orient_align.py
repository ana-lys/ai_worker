#!/usr/bin/env python3
"""Logitech hold-to-align for the right gripper orientation, against the marker.

  printed button 3 (joy index 2), held: rotate the right gripper to RPY 0 0 0
      in the marker frame (EE axes = marker_frame axes)
  printed button 5 (joy index 4), held: rotate it to the saved preset
      (config/ee_orient_presets.yaml, made with --save)

While a button is held the right arm is taken from joy_hand (external_hold) and
driven on the absolute goal path (/quest/right/ee_target_pose): the position
stays where it was at the press, the orientation follows a P controller
(rate = kp * error, capped at wmax) toward the target. Releasing stops it where
it is and hands the arm back; EE soft locks that were on are re-centred on the
new orientation (unlock -> release -> lock, else the stale lock clamps the
re-based goal back). The target is re-read from the board every tick, so a
base move during the hold is followed.

"Marker frame" here is the LEVEL board frame turned by marker_yaw_offset_deg
(= marker_pose_corrector board_yaw_offset, 90 deg): raw ArUco board yaw only,
board roll/pitch ignored (base_link is level).

Save the current right gripper RPY against the marker (1 s average, refused if
the arm or the board is moving):
    ros2 run ffw_spacemouse ee_orient_align.py --save
Run (also started by logitech_teleop.launch.py):
    ros2 run ffw_spacemouse ee_orient_align.py
"""
import argparse, math, os, sys, time
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from ffw_collision_checker.msg import GoalConstraints

PRESETS = os.environ.get('EE_ORIENT_PRESETS') or os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'ee_orient_presets.yaml')


def R_from_q(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def R_from_rpy(r, p, y):
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Rz(y) @ Ry @ Rx


def rpy(R):
    return np.array([math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))),
                     math.atan2(R[1, 0], R[0, 0])])


def rotvec(R):
    """Axis * angle of R."""
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2 * math.sin(ang)) * ang


def R_from_rotvec(v):
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


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


def load_preset():
    try:
        with open(PRESETS) as f:
            d = yaml.safe_load(f) or {}
        return [math.radians(float(v)) for v in d['button5']['rpy_deg']]
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


class Align(Node):
    def __init__(self, name='ee_orient_align'):
        super().__init__(name)
        self.declare_parameter('joy_topic', '/joy_logitech')
        self.declare_parameter('btn_zero', 2)        # printed "3"
        self.declare_parameter('btn_preset', 4)      # printed "5"
        self.declare_parameter('kp', 1.0)            # 1/s
        self.declare_parameter('wmax_deg', 10.0)     # deg/s
        self.declare_parameter('leash_deg', 5.0)     # goal never runs further ahead of the arm
        self.declare_parameter('kpos', 2.0)          # 1/s, integrator holding the arm at the press point
        self.declare_parameter('pos_trim_max', 0.02)  # m, max goal offset that integrator may add
        self.declare_parameter('max_error_deg', 60.0)   # refuse bigger turns
        self.declare_parameter('board_timeout', 0.3)
        self.declare_parameter('marker_yaw_offset_deg', 90.0)
        g = lambda n: self.get_parameter(n).value
        self.btn = {'zero': g('btn_zero'), 'preset': g('btn_preset')}
        self.ach = None             # (t, p, R) solver frame
        self.board = None           # (t, yaw) level board yaw in base_link
        self.locks = None           # (roll, pitch, yaw) EE soft locks of joy_hand
        self.active = None          # 'zero' | 'preset' while held
        self.goal = None            # (p, R) sent on /quest
        self.target_rpy = None
        self.release_at = None      # release sequence time stamps
        self.relock = ()
        self.reached = False
        self.create_subscription(Joy, g('joy_topic'), self._joy, 10)
        self.create_subscription(PoseStamped, '/ik_solver/achieved_ee_pose_r', self._ach, 20)
        self.tgt = None             # (t, p, R) solver's held target, same frame
        self.create_subscription(PoseStamped, '/ik_solver/target_ee_pose_r', self._tgt, 20)
        self.create_subscription(PoseStamped, '/head_camera/marker_board_pose', self._board,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(GoalConstraints, '/spacemouse/right/goal_constraints', self._gc, latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/quest/right/ee_target_pose', 10)
        self.hold_pub = self.create_publisher(Bool, '/spacemouse/right/external_hold', 10)
        self.lock_pub = self.create_publisher(String, '/ik_solver/ee_lock', 10)
        self.dt = 0.02
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'right gripper align: hold joy btn {self.btn["zero"]} -> RPY 0 0 0 vs marker, '
                               f'btn {self.btn["preset"]} -> preset {PRESETS}')

    def _ach(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.ach = (time.monotonic(), np.array([p.x, p.y, p.z]), R_from_q([q.x, q.y, q.z, q.w]))

    def _tgt(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.tgt = (time.monotonic(), np.array([p.x, p.y, p.z]), R_from_q([q.x, q.y, q.z, q.w]))

    def _board(self, m):
        q = m.pose.orientation
        R = R_from_q([q.x, q.y, q.z, q.w])
        self.board = (time.monotonic(), math.atan2(R[1, 0], R[0, 0]))

    def _gc(self, m):
        self.locks = (m.roll_locked, m.pitch_locked, m.yaw_locked)

    def marker_R(self):
        if self.board is None or time.monotonic() - self.board[0] > self.get_parameter('board_timeout').value:
            return None
        return Rz(self.board[1] + math.radians(self.get_parameter('marker_yaw_offset_deg').value))

    def _joy(self, m):
        pressed = [k for k, i in self.btn.items() if len(m.buttons) > i and m.buttons[i]]
        want = pressed[0] if len(pressed) == 1 else None    # both held -> nothing
        if want == self.active:
            return
        if self.active is not None:
            self._release()
        if want is not None and self.release_at is None:
            self._press(want)

    def _press(self, which):
        log = self.get_logger()
        if which == 'preset':
            t = load_preset()
            if t is None:
                log.warn(f'no preset in {PRESETS}: run "ee_orient_align.py --save" first')
                return
        else:
            t = [0.0, 0.0, 0.0]
        Rm = self.marker_R()
        if self.ach is None or time.monotonic() - self.ach[0] > 0.5 or Rm is None:
            log.warn('ignored: no fresh achieved right EE pose or no board detection')
            return
        err = math.degrees(np.linalg.norm(rotvec((Rm @ R_from_rpy(*t)) @ self.ach[2].T)))
        if err > self.get_parameter('max_error_deg').value:
            log.warn(f'ignored: {err:.0f} deg away (max {self.get_parameter("max_error_deg").value:.0f})')
            return
        self.target_rpy = t
        # Start from the target the solver already holds: the IK settles a few
        # mm off its target, so starting from the achieved pose drops the arm.
        fresh = self.tgt is not None and time.monotonic() - self.tgt[0] < 0.5
        src = self.tgt if fresh and np.linalg.norm(self.tgt[1] - self.ach[1]) < 0.03 else self.ach
        self.goal = (src[1].copy(), src[2].copy())
        self.p_start = src[1].copy()
        self.p_hold = self.ach[1].copy()           # where the arm must stay
        self.active = which
        self.reached = False
        self.hold_pub.publish(Bool(data=True))
        log.info(f'{which}: rotating to RPY {np.round(np.degrees(t), 1).tolist()} deg vs marker ({err:.1f} deg away)')

    def _release(self):
        # Unlock BEFORE handing back: joy_hand re-bases on the achieved pose and
        # then clamps it to its EE locks; locks still centred on the old
        # orientation would turn the gripper back. Re-lock them after.
        self.relock = tuple(ax for ax, on in zip(('roll', 'pitch', 'yaw'), self.locks or ()) if on)
        for ax in self.relock:
            self.lock_pub.publish(String(data=f'unlock {ax}_r'))
        self.release_at = time.monotonic()
        self.active = None
        Rm = self.marker_R()
        if Rm is not None and self.ach is not None:
            self.get_logger().info(f'released at RPY {np.round(np.degrees(rpy(Rm.T @ self.ach[2])), 2).tolist()} '
                                   'deg vs marker')

    def _send(self):
        p, R = self.goal
        q = q_from_R(R)
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(v) for v in q)
        self.goal_pub.publish(m)

    def _tick(self):
        now = time.monotonic()
        if self.release_at is not None:
            el = now - self.release_at
            if el < 0.2:                         # unlocks land first
                self.hold_pub.publish(Bool(data=True))
                self._send()
            elif el < 0.26:
                self.hold_pub.publish(Bool(data=False))
            elif el > 0.66:                      # joy_hand re-based on the solver target
                for ax in self.relock:
                    self.lock_pub.publish(String(data=f'lock {ax}_r'))
                self.release_at, self.relock, self.goal = None, (), None
            return
        if self.active is None:
            return
        self.hold_pub.publish(Bool(data=True))   # joy_hand times out after 1 s
        Rm = self.marker_R()
        if Rm is not None and self.ach is not None:
            R_t = Rm @ R_from_rpy(*self.target_rpy)
            # closed loop on the ARM: the error is the achieved orientation's,
            # so a steady IK offset is trimmed out instead of left in
            e = rotvec(R_t @ self.ach[2].T)
            ang = float(np.linalg.norm(e))
            lag = math.degrees(np.linalg.norm(rotvec(self.goal[1] @ self.ach[2].T)))
            Rg = self.goal[1]
            if ang > 1e-6 and lag < self.get_parameter('leash_deg').value:
                step = min(self.get_parameter('kp').value * ang,
                           math.radians(self.get_parameter('wmax_deg').value)) * self.dt
                Rg = R_from_rotvec(e / ang * min(step, ang)) @ Rg
            # position: integrate the arm's drift away from the press point
            pg = self.goal[0] + self.get_parameter('kpos').value * self.dt * (self.p_hold - self.ach[1])
            trim = pg - self.p_start
            lim = self.get_parameter('pos_trim_max').value
            if np.linalg.norm(trim) > lim:
                pg = self.p_start + trim / np.linalg.norm(trim) * lim
            self.goal = (pg, Rg)
            done = math.degrees(np.linalg.norm(rotvec(R_t @ self.ach[2].T))) < 0.3
            if done and not self.reached:
                self.get_logger().info('target reached (arm within 0.3 deg)')
            self.reached = done
        self._send()


def save():
    rclpy.init()
    n = Align('ee_orient_align_save')
    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end and (n.ach is None or n.board is None):
        rclpy.spin_once(n, timeout_sec=0.05)
    Rs, yaws = [], []
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        Rm = n.marker_R()
        if Rm is not None and n.ach is not None:
            Rs.append(Rm.T @ n.ach[2])
            yaws.append(n.board[1])
    rclpy.try_shutdown()
    if len(Rs) < 5:
        sys.exit('not enough samples: need /ik_solver/achieved_ee_pose_r and /head_camera/marker_board_pose')
    U, _, Vt = np.linalg.svd(sum(Rs))
    Rmean = U @ Vt                                 # chordal mean
    spread = max(math.degrees(np.linalg.norm(rotvec(R @ Rmean.T))) for R in Rs)
    if spread > 0.3:
        sys.exit(f'moving (spread {spread:.2f} deg over 1 s): hold still and retry')
    r = [round(float(v), 3) for v in np.degrees(rpy(Rmean))]
    os.makedirs(os.path.dirname(PRESETS), exist_ok=True)
    with open(PRESETS, 'w') as f:
        f.write('# right gripper RPY [deg] in the level marker frame (ee_orient_align.py --save)\n')
        yaml.safe_dump({'button5': {'rpy_deg': r, 'saved': time.strftime('%Y-%m-%d %H:%M:%S'),
                                    'samples': len(Rs), 'spread_deg': round(spread, 3)}}, f)
    print(f'saved RPY {r} deg (spread {spread:.2f} deg, {len(Rs)} samples) -> {os.path.realpath(PRESETS)}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--save', action='store_true', help='save the current right gripper RPY vs the marker')
    a, _ = ap.parse_known_args()
    if a.save:
        return save()
    rclpy.init()
    n = Align()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.active is not None:
            n.hold_pub.publish(Bool(data=False))
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
