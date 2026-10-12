#!/usr/bin/env python3
"""Scripted check of joy_hand_marker (right arm) on an isolated domain.

Publishes a live base_link -> marker_frame TF (the "EKF") and moves it to
simulate base motion; checks the published base-frame goal stays fixed in
marker_frame, deltas act in marker axes, and TF outages hold + re-anchor.
"""
import json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, TransformStamped
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
        super().__init__('marker_regress')
        self.goals = []
        self.create_subscription(PoseStamped, '/spacemouse/right/ee_target_pose', self.on_goal, 100)
        self.joy = self.create_publisher(Joy, '/right/joy', 10)
        self.mode = self.create_publisher(String, '/teleop_mode', 10)
        self.ach = self.create_publisher(PoseStamped, '/ik_solver/achieved_ee_pose_r', 10)
        self.tfb = TransformBroadcaster(self)
        self.achieved = P0.copy()
        self.axes = [0.0] * 6
        self.T_bm = T(1.2, 0.0, 0.0)  # base_link <- marker_frame
        self.tf_on = True
        self.tf_stamp_lag = 0.0

    def on_goal(self, m):
        p = m.pose.position
        self.goals.append(np.array([p.x, p.y, p.z]))

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            now = self.get_clock().now()
            j = Joy(); j.header.stamp = now.to_msg(); j.axes = list(self.axes); j.buttons = [0, 0]
            self.joy.publish(j)
            a = PoseStamped(); a.header.stamp = now.to_msg(); a.header.frame_id = 'map'
            a.pose.position.x, a.pose.position.y, a.pose.position.z = self.achieved
            a.pose.orientation.w = 1.0
            self.ach.publish(a)
            if self.tf_on:
                t = TransformStamped()
                t.header.stamp = (now - rclpy.duration.Duration(seconds=self.tf_stamp_lag)).to_msg()
                t.header.frame_id = 'base_link'; t.child_frame_id = 'marker_frame'
                t.transform.translation.x, t.transform.translation.y = self.T_bm[0, 3], self.T_bm[1, 3]
                yaw = math.atan2(self.T_bm[1, 0], self.T_bm[0, 0])
                t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                self.tfb.sendTransform(t)
            rclpy.spin_once(self, timeout_sec=0.01)

    def take(self):
        g, self.goals = self.goals, []
        return g


def mm(v):
    return [round(float(x) * 1000, 1) for x in v]


def main():
    rclpy.init()
    h = H()
    out = {}
    t0 = time.time()
    while time.time() - t0 < 10.0 and not all(p.get_subscription_count() > 0 for p in (h.joy, h.mode, h.ach)):
        h.pump(0.1)
    h.pump(1.0)
    t0 = time.time()
    while not h.goals and time.time() - t0 < 5.0:
        h.mode.publish(String(data='ARM')); h.pump(0.1)
    h.pump(0.5)
    out['anchor_mm'] = mm(h.take()[-1] - P0)  # expect 0
    # base drives 10 cm toward the board and yaws 0.1 rad (marker moves in base_link)
    T0 = h.T_bm.copy()
    h.T_bm = T(1.1, 0.02, 0.1)
    h.pump(0.5)
    expect = (h.T_bm @ np.linalg.inv(T0) @ np.append(P0, 1))[:3]
    out['base_move_err_mm'] = mm(h.take()[-1] - expect)  # expect 0: EE fixed in marker
    # SpaceMouse +x with the board yawed 90 deg in base_link: motion is along marker x = base +y
    h.T_bm = T(1.1, 0.02, math.pi / 2)
    h.pump(0.3); h.take()
    h.axes = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
    h.pump(0.5)
    g = h.take()
    h.axes = [0.0] * 6
    d = g[-1] - g[0]
    out['marker_x_dir_base'] = [round(float(x), 2) for x in d / np.linalg.norm(d)]  # expect [0, 1, 0]
    # stale TF (stamps 0.5 s old) -> hold
    h.pump(0.3); h.take()
    h.tf_stamp_lag = 0.5
    h.pump(0.3); h.take()
    h.pump(0.5)
    out['stale_goals'] = len(h.take())  # expect 0
    h.tf_stamp_lag = 0.0
    # TF outage while the base moves and the arm drifts: no goals; on recovery
    # the goal re-anchors at the achieved pose (no jump back)
    h.tf_on = False
    h.pump(0.6); h.take()
    h.pump(0.5)
    out['outage_goals'] = len(h.take())  # expect 0
    h.achieved = P0 + np.array([0.03, 0.0, 0.0])
    h.T_bm = T(1.0, 0.0, 0.0)
    h.tf_on = True
    h.pump(0.5)
    out['recover_err_mm'] = mm(h.take()[-1] - h.achieved)  # expect 0
    # BASE mode: joy_hand is silent; whoever anchors must keep the goal on the board
    h.mode.publish(String(data='BASE')); h.pump(0.3); h.take()
    T1 = h.T_bm.copy()
    g_before = None
    h.pump(0.3)
    g = h.take()
    g_before = g[-1] if g else None
    h.T_bm = T(0.95, -0.03, -0.05)
    h.pump(0.5)
    g = h.take()
    if g_before is None or not g:
        out['base_mode_anchor_err_mm'] = None
    else:
        expect = (h.T_bm @ np.linalg.inv(T1) @ np.append(g_before, 1))[:3]
        out['base_mode_anchor_err_mm'] = mm(g[-1] - expect)
    print(json.dumps(out, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
