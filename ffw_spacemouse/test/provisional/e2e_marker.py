#!/usr/bin/env python3
"""End to end, sim: marker_frame_teleop.launch.py (hardware_mode:=false) with
the real IK solver. Fake EKF TF + board detections; moving the board in
base_link must move the solver's achieved EEs with it."""
import json, math, subprocess, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class H(Node):
    def __init__(self):
        super().__init__('e2e_marker')
        self.ach = {}
        for a in ('l', 'r'):
            self.create_subscription(PoseStamped, f'/ik_solver/achieved_ee_pose_{a}',
                                     lambda m, a=a: self.ach.__setitem__(a, np.array(
                                         [m.pose.position.x, m.pose.position.y, m.pose.position.z])), 10)
        self.goal_r = None
        self.create_subscription(PoseStamped, '/spacemouse/right/ee_target_pose',
                                 lambda m: setattr(self, 'goal_r', np.array(
                                     [m.pose.position.x, m.pose.position.y, m.pose.position.z])), 10)
        self.halted = None
        self.create_subscription(Bool, '/teleop_core/halted', lambda m: setattr(self, 'halted', m.data),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.det = self.create_publisher(PoseStamped, '/head_camera/marker_board_pose', 10)
        self.tfb = TransformBroadcaster(self)
        self.tx = 1.2
        self.k = 0

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            self.k += 1
            now = self.get_clock().now()
            t = TransformStamped(); t.header.stamp = now.to_msg()
            t.header.frame_id = 'base_link'; t.child_frame_id = 'marker_frame'
            t.transform.translation.x = self.tx; t.transform.rotation.w = 1.0
            self.tfb.sendTransform(t)
            if self.k % 3 == 0:
                d = PoseStamped(); d.header.stamp = now.to_msg(); d.header.frame_id = 'base_link'
                self.det.publish(d)
            time.sleep(0.02)
            rclpy.spin_once(self, timeout_sec=0.01)


def param(node, name):
    for _ in range(3):
        res = subprocess.run(['ros2', 'param', 'get', node, name], capture_output=True, text=True, timeout=20)
        if res.stdout.strip():
            return res.stdout.strip().split(':')[-1].strip()
        err = res.stderr.strip()
        time.sleep(1.0)
    return 'ERR ' + err[-120:]


def main():
    rclpy.init()
    h = H()
    r = {}
    t0 = time.time()
    while time.time() - t0 < 40 and not (h.halted is False and 'l' in h.ach and 'r' in h.ach):
        h.pump(0.2)
    r['resumed'] = h.halted is False
    r['achieved_seen'] = 'l' in h.ach and 'r' in h.ach
    h.pump(2.0)   # joy_hand_marker re-bases and anchors
    a0 = {k: v.copy() for k, v in h.ach.items()}
    g0 = h.goal_r.copy()
    # base drives 5 cm toward the board over 1 s -> EEs must move -5 cm in x
    for i in range(50):
        h.tx = 1.2 - 0.05 * (i + 1) / 50
        h.pump(0.02)
    trace = []
    for _ in range(12):
        h.pump(0.5)
        trace.append(round(float(h.ach['r'][0] - a0['r'][0]) * 1000, 1))
    r['right_x_shift_mm_every_0.5s'] = trace
    r['right_goal_shift_mm'] = [round(float(x) * 1000, 1) for x in h.goal_r - g0]
    r['right_goal_minus_achieved_mm'] = [round(float(x) * 1000, 1) for x in h.goal_r - h.ach['r']]
    r['ee_shift_mm'] = {k: [round(float(x) * 1000) for x in h.ach[k] - a0[k]] for k in a0} if a0 else None
    r['params'] = {
        'teleop_core.marker_health': param('/teleop_core', 'marker_health'),
        'solver.joy_lift': param('/ffw_ik_solver_teleop', 'joy_lift'),
        'right_mapper.use_teleop_core': param('/right/joy_hand', 'use_teleop_core'),
        'joy_base.use_teleop_core': param('/joy_base', 'use_teleop_core'),
    }
    print(json.dumps(r, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
