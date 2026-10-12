#!/usr/bin/env python3
"""teleop_core marker health gate (D4) with joy_hand_marker as the arm source."""
import json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster

P0 = np.array([0.30, -0.20, 1.00])


def T(x, y, yaw):
    M = np.eye(4)
    M[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    M[:3, 3] = [x, y, 0.0]
    return M


class H(Node):
    def __init__(self):
        super().__init__('health_regress')
        self.goals, self.cmds, self.halted = [], [], []
        self.create_subscription(PoseStamped, '/spacemouse/right/ee_target_pose',
                                 lambda m: self.goals.append(np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])), 100)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.cmds.append((m.linear.x, m.angular.z)), 100)
        self.create_subscription(Bool, '/teleop_core/halted', lambda m: self.halted.append(m.data),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.joy = self.create_publisher(Joy, '/right/joy', 10)
        self.mode = self.create_publisher(String, '/teleop_mode', 10)
        self.ach = self.create_publisher(PoseStamped, '/ik_solver/achieved_ee_pose_r', 10)
        self.det = self.create_publisher(PoseStamped, '/head_camera/marker_board_pose', 10)
        self.sm = self.create_publisher(Twist, '/teleop_core/base/spacemouse', 10)
        self.tfb = TransformBroadcaster(self)
        self.achieved = P0.copy()
        self.T_bm = T(1.2, 0.0, 0.0)
        self.det_on = True
        self.sm_cmd = None
        self.k = 0

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            now = self.get_clock().now()
            self.k += 1
            j = Joy(); j.header.stamp = now.to_msg(); j.axes = [0.0] * 6; j.buttons = [0, 0]
            self.joy.publish(j)
            a = PoseStamped(); a.header.stamp = now.to_msg(); a.header.frame_id = 'map'
            a.pose.position.x, a.pose.position.y, a.pose.position.z = self.achieved
            a.pose.orientation.w = 1.0
            self.ach.publish(a)
            if self.det_on and self.k % 10 == 0:   # ~10 Hz detections
                d = PoseStamped(); d.header.stamp = now.to_msg(); d.header.frame_id = 'base_link'
                self.det.publish(d)
            if self.k % 4 == 0:                    # ~25 Hz EKF TF
                t = TransformStamped(); t.header.stamp = now.to_msg()
                t.header.frame_id = 'base_link'; t.child_frame_id = 'marker_frame'
                t.transform.translation.x, t.transform.translation.y = self.T_bm[0, 3], self.T_bm[1, 3]
                yaw = math.atan2(self.T_bm[1, 0], self.T_bm[0, 0])
                t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                self.tfb.sendTransform(t)
            if self.sm_cmd is not None:
                self.sm.publish(self.sm_cmd)
            time.sleep(0.008)
            rclpy.spin_once(self, timeout_sec=0.002)

    def take(self):
        g, self.goals = self.goals, []
        return g


def mm(v):
    return [round(float(x) * 1000, 1) for x in v]


def main():
    rclpy.init()
    h = H()
    r = {}
    t0 = time.time()
    while time.time() - t0 < 10 and not all(p.get_subscription_count() > 0 for p in (h.joy, h.mode, h.ach, h.det, h.sm)):
        h.pump(0.1)
    for _ in range(5):
        h.mode.publish(String(data='ARM')); h.pump(0.05)
    r['starts_halted'] = bool(h.halted) and h.halted[0] is True
    h.take()
    h.pump(0.5)
    r['no_goals_while_suspended'] = len(h.take()) == 0 or not h.halted[-1]
    h.pump(1.5)
    r['resumed'] = h.halted[-1] is False
    h.pump(0.3)
    g = h.take()
    r['anchor_mm'] = mm(g[-1] - P0) if g else None
    # base moves under the SpaceMouse (capped) -> goal follows the board
    h.sm_cmd = Twist(); h.sm_cmd.linear.x = 0.5
    h.pump(0.5)
    r['cmd_capped'] = round(h.cmds[-1][0], 3) if h.cmds else None       # 0.1
    h.sm_cmd = Twist()
    # detections stop -> HALT after 2 s; base zero; goal frozen while the board "moves"
    h.det_on = False
    h.pump(2.3)
    r['halt_on_detection_loss'] = h.halted[-1] is True
    h.take(); h.cmds.clear()
    h.sm_cmd = Twist(); h.sm_cmd.linear.x = 0.5
    T_before = h.T_bm.copy()
    h.T_bm = T(1.15, 0.05, 0.05)
    h.pump(0.5)
    g = h.take()
    r['halted_cmd_zero'] = bool(h.cmds) and all(c == (0.0, 0.0) for c in h.cmds)
    r['halted_goals_moving'] = len(g) > 1 and float(np.ptp(np.array(g), axis=0).max()) > 1e-6
    h.sm_cmd = None
    # resume: joy_hand re-bases onto the achieved pose, then the goal is anchored again
    h.achieved = P0 + np.array([0.01, 0.0, 0.0])
    h.det_on = True
    h.pump(1.5)
    r['resumed_after_loss'] = h.halted[-1] is False
    h.take()
    h.pump(0.3)
    g = h.take()
    r['resume_goal_minus_achieved_mm'] = mm(g[-1] - h.achieved) if g else None
    T1 = h.T_bm.copy(); gb = g[-1]
    h.T_bm = T(1.10, 0.05, 0.05)
    h.pump(0.4)
    g = h.take()
    exp = (h.T_bm @ np.linalg.inv(T1) @ np.append(gb, 1))[:3]
    r['reanchored_follow_err_mm'] = mm(g[-1] - exp) if g else None
    # EKF jump of 10 cm -> immediate HALT
    h.T_bm = T(1.0, 0.05, 0.05)
    h.pump(0.3)
    r['halt_on_jump'] = h.halted[-1] is True
    print(json.dumps(r, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
