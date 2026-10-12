#!/usr/bin/env python3
"""Scripted regression run for one joy_hand instance (right arm).

Run joy_hand under namespace /right on an ISOLATED domain, then this script.
Prints a JSON summary of what joy_hand published per phase; diff two runs.
"""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Bool

P0 = (0.30, -0.20, 1.00)


def rpy(q):
    x, y, z, w = q
    # extrinsic XYZ (R = Rx Ry Rz), same as joy_hand extract_rpy
    R02 = 2 * (x * z + w * y)
    R01 = 2 * (x * y - w * z); R00 = 1 - 2 * (y * y + z * z)
    R12 = 2 * (y * z - w * x); R22 = 1 - 2 * (x * x + y * y)
    return (math.atan2(-R12, R22), math.asin(max(-1, min(1, R02))), math.atan2(-R01, R00))


class H(Node):
    def __init__(self):
        super().__init__('joy_hand_regress')
        self.goals = []
        self.deltas = []
        self.create_subscription(PoseStamped, '/spacemouse/right/ee_target_pose', self.on_goal, 100)
        self.create_subscription(PoseStamped, '/quest/right/ee_target_pose', self.on_quest, 100)
        self.create_subscription(TwistStamped, '/spacemouse/right/ee_target_delta',
                                 lambda m: self.deltas.append(m), 100)
        self.quest = 0
        self.joy = self.create_publisher(Joy, '/right/joy', 10)
        self.mode = self.create_publisher(String, '/teleop_mode', 10)
        self.ach = self.create_publisher(PoseStamped, '/ik_solver/achieved_ee_pose_r', 10)
        self.lim = self.create_publisher(String, '/teleop/limit_profile', 10)
        self.lock = self.create_publisher(String, '/ik_solver/ee_lock', 10)
        self.hold = self.create_publisher(Bool, '/spacemouse/right/external_hold', 10)
        self.achieved = P0
        self.axes = [0.0] * 6

    def on_goal(self, m):
        p = m.pose.position; o = m.pose.orientation
        self.goals.append(((p.x, p.y, p.z), rpy((o.x, o.y, o.z, o.w)), m.header.frame_id))

    def on_quest(self, m):
        self.quest += 1

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            j = Joy(); j.header.stamp = self.get_clock().now().to_msg(); j.axes = list(self.axes); j.buttons = [0, 0]
            self.joy.publish(j)
            a = PoseStamped(); a.header.stamp = self.get_clock().now().to_msg(); a.header.frame_id = 'map'
            a.pose.position.x, a.pose.position.y, a.pose.position.z = self.achieved
            a.pose.orientation.w = 1.0
            self.ach.publish(a)
            rclpy.spin_once(self, timeout_sec=0.01)

    def take(self):
        g, self.goals = self.goals, []
        return g


def r(v, n=4):
    return [round(x, n) for x in v]


def main():
    rclpy.init()
    h = H()
    out = {}
    # discovery: every endpoint matched before anything is sent
    t0 = time.time()
    while time.time() - t0 < 10.0 and not all(
            p.get_subscription_count() > 0 for p in (h.joy, h.mode, h.ach, h.lim, h.lock, h.hold)):
        h.pump(0.1)
    h.pump(1.0)  # mode still BASE
    out['base_mode_goals'] = len(h.take())
    t0 = time.time()
    while not h.goals and time.time() - t0 < 5.0:
        h.mode.publish(String(data='ARM')); h.pump(0.1)
    h.pump(1.0)
    g = h.take()
    out['idle_n>0'] = len(g) > 50
    out['idle_last'] = r(g[-1][0]) if g else None
    out['frame'] = g[-1][2] if g else None
    # translate +x (axis index 1 = axis_x in the mapper launch)
    h.axes = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
    h.pump(1.0)
    g = h.take()
    steps = [g[i + 1][0][0] - g[i][0][0] for i in range(len(g) - 1)]
    out['x_step_median_mm'] = round(sorted(steps)[len(steps) // 2] * 1000, 4)
    # yaw rotation (axis 5)
    h.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
    h.pump(0.5)
    g = h.take()
    ys = [g[i + 1][1][2] - g[i][1][2] for i in range(len(g) - 1)]
    out['yaw_step_median_mrad'] = round(sorted(ys)[len(ys) // 2] * 1000, 4)
    yaw_at_lock = g[-1][1][2]
    # yaw lock: further yaw input must stay within 1 deg of the lock center
    h.lock.publish(String(data='lock yaw_r')); h.pump(0.2)
    h.take()
    h.pump(1.0)
    g = h.take()
    # spread while locked with yaw input held (0 = pinned at the slack bound);
    # the lock center itself depends on message latency, so it is not compared
    out['yaw_lock_spread_deg'] = round((max(x[1][2] for x in g) - min(x[1][2] for x in g)) * 180 / math.pi, 3)
    out['yaw_lock_dev_le_1.2deg'] = max(abs(x[1][2] - yaw_at_lock) for x in g) * 180 / math.pi < 1.2
    h.lock.publish(String(data='unlock yaw_r'))
    # limit profile: x max at P0.x + 0.05 while pushing +x
    h.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    h.lim.publish(String(data='set r -9 %.3f -9 9 -9 9 -9 9 -9 9 -9 9' % (P0[0] + 0.05)))
    h.pump(1.5)
    g = h.take()
    out['limit_x_final'] = round(g[-1][0][0], 4)
    h.lim.publish(String(data='clear r'))
    h.axes = [0.0] * 6
    h.pump(0.3); h.take()
    # external hold: silent while held, rebased to achieved after release
    h.achieved = (0.35, -0.15, 0.95)
    t_end = time.time() + 1.0
    while time.time() < t_end:
        h.hold.publish(Bool(data=True)); h.pump(0.1)
    h.pump(0.1)  # drain the last in-flight ticks before counting
    h.take()
    for _ in range(5):
        h.hold.publish(Bool(data=True)); h.pump(0.1)
    out['held_goals'] = len(h.take())
    h.hold.publish(Bool(data=False)); h.pump(0.5)
    g = h.take()
    out['after_release'] = r(g[-1][0]) if g else None
    # BASE -> ARM rebases onto a new achieved pose
    h.mode.publish(String(data='BASE')); h.pump(0.3); h.take()
    h.pump(0.5)
    out['base_goals'] = len(h.take())
    h.achieved = (0.25, -0.25, 1.05)
    h.pump(0.2)
    h.mode.publish(String(data='ARM')); h.pump(0.5)
    g = h.take()
    out['after_arm_switch'] = r(g[-1][0]) if g else None
    out['quest_topic_msgs'] = h.quest
    out['delta_msgs>0'] = len(h.deltas) > 0
    print(json.dumps(out, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
