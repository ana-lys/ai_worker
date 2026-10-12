#!/usr/bin/env python3
"""teleop_core lift ramp (D7) + head relay, with synthetic goals."""
import json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformBroadcaster


class H(Node):
    def __init__(self):
        super().__init__('lift_head_regress')
        self.goals = {'left': [], 'right': []}
        for arm in ('left', 'right'):
            self.create_subscription(PoseStamped, f'/spacemouse/{arm}/ee_target_pose',
                                     lambda m, a=arm: self.goals[a].append(
                                         np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])), 100)
        self.heads = []
        self.create_subscription(JointTrajectory, '/leader/joystick_controller_left/joint_trajectory',
                                 lambda m: self.heads.append(list(m.points[0].positions)), 100)
        self.goal = {a: self.create_publisher(PoseStamped, f'/teleop_core/{a}/ee_goal', 10) for a in ('left', 'right')}
        self.mode = self.create_publisher(String, '/teleop_mode', 10)
        self.lift_sm = self.create_publisher(Float64, '/teleop_core/lift/spacemouse', 10)
        self.lift_lg = self.create_publisher(Float64, '/teleop_core/lift/logitech', 10)
        self.head_sm = self.create_publisher(JointTrajectory, '/teleop_core/head/spacemouse', 10)
        self.head_lg = self.create_publisher(JointTrajectory, '/teleop_core/head/logitech', 10)
        self.tfb = TransformBroadcaster(self)
        self.tx = 1.2

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'; t.child_frame_id = 'marker_frame'
            t.transform.translation.x = self.tx; t.transform.rotation.w = 1.0
            self.tfb.sendTransform(t)
            rclpy.spin_once(self, timeout_sec=0.01)

    def send_goal(self, arm, frame, p):
        g = PoseStamped(); g.header.frame_id = frame; g.pose.orientation.w = 1.0
        g.pose.position.x, g.pose.position.y, g.pose.position.z = p
        self.goal[arm].publish(g)

    def take(self, arm):
        g, self.goals[arm] = self.goals[arm], []
        return g


def head(p):
    m = JointTrajectory(); m.joint_names = ['head_joint1', 'head_joint2']
    pt = JointTrajectoryPoint(); pt.positions = list(p); m.points.append(pt)
    return m


def main():
    rclpy.init()
    h = H()
    r = {}
    t0 = time.time()
    while time.time() - t0 < 10 and not all(p.get_subscription_count() for p in
                                            (h.mode, h.lift_sm, h.lift_lg, h.head_sm, h.head_lg, h.goal['left'])):
        h.pump(0.1)
    h.pump(0.3)
    # left: base_link goal, right: anchored marker goal (base x 0.4)
    h.send_goal('left', 'base_link', (0.4, 0.3, 1.0))
    h.send_goal('right', 'marker_frame', (-0.8, -0.3, 1.0))
    h.pump(0.3); h.take('left'); h.take('right')
    # ARM mode: lift ignored
    h.mode.publish(String(data='ARM')); h.pump(0.1)
    h.lift_lg.publish(Float64(data=0.1)); h.pump(0.5)
    r['arm_mode_lift_ignored'] = len(h.take('left')) == 0
    # BASE: logitech -0.10 then spacemouse +0.04 -> net -0.06, ramped
    h.mode.publish(String(data='BASE')); h.pump(0.2); h.take('right')
    h.lift_lg.publish(Float64(data=-0.10)); h.lift_sm.publish(Float64(data=0.04))
    h.pump(2.0)
    gl, gr = h.take('left'), h.take('right')
    zl = [g[2] for g in gl]
    r['left_z_final'] = round(zl[-1], 4)                                  # 0.94
    r['left_max_step_mm'] = round(max(abs(b - a) for a, b in zip([1.0] + zl, zl)) * 1000, 3)  # <= 0.5
    r['right_z_final'] = round(gr[-1][2], 4)                              # 0.94
    # anchored right still follows the board after the lift
    h.tx = 1.1; h.pump(0.3)
    gr = h.take('right')
    r['right_follows_board'] = [round(v, 4) for v in gr[-1]]              # [0.3, -0.3, 0.94]
    # reach box z (right max 1.60): +1.0 stops at 1.60 for both
    h.lift_sm.publish(Float64(data=1.0)); h.pump(14.0)
    r['right_z_box_stop'] = round(h.take('right')[-1][2], 4)              # 1.6
    r['left_z_box_stop'] = round(h.take('left')[-1][2], 4)                # 1.6 (stops with the anchored one)
    # head relay: spacemouse passes; logitech blocked within 0.3 s of it, passes after
    h.heads.clear()
    h.head_sm.publish(head((0.1, 0.2))); h.pump(0.05)
    h.head_lg.publish(head((0.3, 0.4))); h.pump(0.5)
    h.head_lg.publish(head((0.5, 0.6))); h.pump(0.2)
    r['heads'] = [[round(v, 2) for v in x] for x in h.heads]             # [[0.1,0.2],[0.5,0.6]]
    print(json.dumps(r, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
