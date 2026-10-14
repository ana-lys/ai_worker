#!/usr/bin/env python3
"""ee_orient_align.py: hold-to-rotate the right gripper against the marker.
This harness is the arm: the solver target follows the /quest goal and the
achieved pose settles OFF it like the real IK (+1.3 -0.9 -3.5 mm, 1 deg about
x, 0.1 s lag); it publishes both,
the ArUco board, joy_hand's latched goal constraints (yaw locked) and the
Logitech. It starts ee_orient_align itself with a temporary preset file."""
import json, math, os, subprocess, sys, tempfile, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from ffw_collision_checker.msg import GoalConstraints

sys.path.insert(0, os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_spacemouse/scripts'))
from ee_orient_align import R_from_q, q_from_R, Rz, R_from_rpy, rpy, rotvec, R_from_rotvec  # noqa: E402

BOARD_YAW = math.radians(-98.78)
RM = Rz(BOARD_YAW + math.pi / 2)
P0 = np.array([0.55, -0.25, 0.95])
BIAS_P = np.array([0.0013, -0.0009, -0.0035])        # achieved - target, real robot 2026-10-14
BIAS_R = R_from_rotvec(np.radians([1.0, 0.0, 0.0]))


class W(Node):
    def __init__(self):
        super().__init__('orient_align_regress')
        self.R = RM @ R_from_rpy(*np.radians([4.0, -6.0, 8.0]))
        self.p = P0.copy()
        self.tp, self.tR = P0 - BIAS_P, BIAS_R.T @ self.R     # target the solver holds
        self.goal = None
        self.events = []          # ('hold', bool) / ('lock', str), changes only
        self.hold = False
        self.goal_n = 0
        self.ach = self.create_publisher(PoseStamped, '/ik_solver/achieved_ee_pose_r', 20)
        self.tgt = self.create_publisher(PoseStamped, '/ik_solver/target_ee_pose_r', 20)
        self.board = self.create_publisher(PoseStamped, '/head_camera/marker_board_pose', qos_profile_sensor_data)
        self.joy = self.create_publisher(Joy, '/joy_logitech', 10)
        gc = self.create_publisher(GoalConstraints, '/spacemouse/right/goal_constraints',
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                              reliability=ReliabilityPolicy.RELIABLE))
        m = GoalConstraints(); m.arm = 'right'; m.yaw_locked = True
        gc.publish(m)
        self.create_subscription(PoseStamped, '/quest/right/ee_target_pose', self._goal, 20)
        self.create_subscription(Bool, '/spacemouse/right/external_hold', self._hold, 20)
        self.create_subscription(String, '/ik_solver/ee_lock', lambda m: self.events.append(('lock', m.data)), 20)
        self.k = 0
        self.create_timer(0.02, self.step)

    def _goal(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.goal = (np.array([p.x, p.y, p.z]), R_from_q([q.x, q.y, q.z, q.w]))
        self.goal_n += 1

    def _hold(self, m):
        if m.data != self.hold:
            self.hold = m.data
            self.events.append(('hold', m.data))

    def step(self):
        if self.goal is not None and self.hold:
            self.tp, self.tR = self.goal
        self.p += (self.tp + BIAS_P - self.p) * 0.2
        self.R = R_from_rotvec(rotvec(BIAS_R @ self.tR @ self.R.T) * 0.2) @ self.R
        st = self.get_clock().now().to_msg()
        for pub, p, R in ((self.ach, self.p, self.R), (self.tgt, self.tp, self.tR)):
            a = PoseStamped(); a.header.stamp = st; a.header.frame_id = 'map'
            a.pose.position.x, a.pose.position.y, a.pose.position.z = (float(v) for v in p)
            q = q_from_R(R)
            a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w = (float(v) for v in q)
            pub.publish(a)
        self.k += 1
        if self.k % 5 == 0:
            b = PoseStamped(); b.header.stamp = st; b.header.frame_id = 'base_link'
            b.pose.position.x, b.pose.position.z = 0.8, 0.76
            b.pose.orientation.z, b.pose.orientation.w = math.sin(BOARD_YAW / 2), math.cos(BOARD_YAW / 2)
            self.board.publish(b)

    def press(self, idx, secs):
        end = time.time() + secs
        while time.time() < end:
            j = Joy(); j.buttons = [0] * 12
            for i in idx:
                j.buttons[i] = 1
            self.joy.publish(j)
            rclpy.spin_once(self, timeout_sec=0.01)

    def rel(self):
        return [round(float(v), 1) for v in np.degrees(rpy(RM.T @ self.R))]

    def drift_mm(self):
        return round(float(np.linalg.norm(self.p - P0)) * 1000, 1)


def main():
    rclpy.init()
    w = W()
    env = dict(os.environ, EE_ORIENT_PRESETS=os.path.join(tempfile.mkdtemp(), 'presets.yaml'))
    lib = subprocess.check_output(['ros2', 'pkg', 'prefix', 'ffw_spacemouse'], text=True).strip() + '/lib/ffw_spacemouse'
    proc = subprocess.Popen([lib + '/ee_orient_align.py'], env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    r = {}
    try:
        t0 = time.time()
        while time.time() - t0 < 10 and not (w.joy.get_subscription_count() and w.ach.get_subscription_count()):
            rclpy.spin_once(w, timeout_sec=0.05)
        w.press([], 1.0)
        # preset button without a preset: refused, arm stays with joy_hand
        w.press([4], 0.5); w.press([], 0.3)
        r['no_preset_refused'] = not w.events
        # save the current orientation as the preset
        sv = subprocess.Popen([lib + '/ee_orient_align.py', '--save'], env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True)
        while sv.poll() is None:
            rclpy.spin_once(w, timeout_sec=0.01)
        r['saved'] = 'saved RPY' in sv.stdout.read()
        r['start_rpy'] = w.rel()                                   # [4, -6, 8]
        # hold button 3 (idx 2): to 0 0 0, position kept
        w.events.clear()
        w.press([2], 1.0)
        r['after_1s_rpy'] = w.rel()                                # partway (rate capped)
        w.press([2], 6.0)
        r['zero_rpy'] = w.rel()                                    # [0, 0, 0]
        r['zero_pos_drift_mm'] = w.drift_mm()                     # ~0 (IK offset trimmed out)
        w.press([], 1.5)
        r['zero_events'] = w.events[:]     # hold on, unlock yaw, hold off, lock yaw
        r['released_rpy'] = w.rel()
        # hold button 5 (idx 4): back to the saved preset
        w.events.clear()
        w.press([4], 7.0); w.press([], 1.5)
        r['preset_rpy'] = w.rel()                                  # [4, -6, 8]
        r['preset_pos_drift_mm'] = w.drift_mm()
        # both buttons: nothing
        w.events.clear()
        w.press([2, 4], 1.0); w.press([], 0.5)
        r['both_ignored'] = not w.events
    finally:
        proc.terminate(); proc.wait(timeout=5)
    print(json.dumps(r, indent=1, sort_keys=True))
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
