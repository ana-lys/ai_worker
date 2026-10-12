#!/usr/bin/env python3
"""Scripted check of teleop_core's /cmd_vel merge + anchored speed cap."""
import json, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


def tw(x=0.0, y=0.0, wz=0.0):
    t = Twist(); t.linear.x, t.linear.y, t.angular.z = x, y, wz
    return t


class H(Node):
    def __init__(self):
        super().__init__('base_regress')
        self.out = []
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.out.append(
            (time.time(), m.linear.x, m.linear.y, m.angular.z)), 100)
        self.sm = self.create_publisher(Twist, '/teleop_core/base/spacemouse', 10)
        self.lg = self.create_publisher(Twist, '/teleop_core/base/logitech', 10)
        self.goal = self.create_publisher(PoseStamped, '/teleop_core/right/ee_goal', 10)
        self.tfb = TransformBroadcaster(self)
        self.sm_cmd = None
        self.lg_cmd = None
        self.tf_on = False

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            if self.sm_cmd is not None: self.sm.publish(self.sm_cmd)
            if self.lg_cmd is not None: self.lg.publish(self.lg_cmd)
            if self.tf_on:
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'; t.child_frame_id = 'marker_frame'
                t.transform.translation.x = 1.2; t.transform.rotation.w = 1.0
                self.tfb.sendTransform(t)
            rclpy.spin_once(self, timeout_sec=0.01)

    def take(self):
        o, self.out = self.out, []
        return o


def last(o):
    return [round(v, 4) for v in o[-1][1:]] if o else None


def main():
    rclpy.init()
    h = H()
    r = {}
    t0 = time.time()
    while time.time() - t0 < 10 and not (h.sm.get_subscription_count() and h.lg.get_subscription_count()
                                         and h.count_publishers('/cmd_vel')):
        h.pump(0.1)
    h.pump(0.5)
    r['silent_when_no_input'] = len(h.take()) == 0
    h.sm_cmd = tw(0.5, 0.0, 0.3); h.pump(0.5)
    r['sm_only'] = last(h.take())                       # [0.5, 0, 0.3]
    h.lg_cmd = tw(0.0, 0.4, 0.0); h.pump(0.5)
    r['sm_beats_logitech'] = last(h.take())             # [0.5, 0, 0.3]
    h.sm_cmd = tw(); h.pump(0.15)
    r['sm_zero_first_0.15s'] = last(h.take())           # zeros (SM just stopped)
    h.pump(0.4)
    r['logitech_after_idle'] = last(h.take())           # [0, 0.4, 0]
    h.sm_cmd = tw(0.2); h.pump(0.2)
    r['sm_takes_back'] = last(h.take())                 # [0.2, 0, 0]
    h.sm_cmd = None; h.pump(0.6)
    r['sm_stale_logitech'] = last(h.take())             # [0, 0.4, 0]
    h.lg_cmd = None; h.pump(0.6)
    o = h.take()
    tail = [x for x in o if x[0] > time.time() - 0.25]
    r['stop_then_silence'] = (last(o) == [0.0, 0.0, 0.0], len(tail) == 0)
    # anchored goal -> uniform speed cap
    h.tf_on = True
    g = PoseStamped(); g.header.frame_id = 'marker_frame'; g.pose.orientation.w = 1.0
    g.pose.position.x = -0.8
    h.goal.publish(g); h.pump(0.3)
    h.lg_cmd = tw(0.3, 0.3, 0.1); h.pump(0.5)
    o = last(h.take())
    r['anchored_capped'] = o                            # |v| 0.1, ratios kept
    r['anchored_speed'] = round(math.hypot(o[0], o[1]), 4) if o else None
    print(json.dumps(r, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
