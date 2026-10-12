#!/usr/bin/env python3
"""teleop_core base clamp (D5), closed loop: the fake base integrates /cmd_vel
into the base_link -> marker_frame TF; the fake arm tracks the goal unless
told to lag."""
import json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from tf2_ros import TransformBroadcaster


def T(x, y, yaw):
    M = np.eye(4)
    M[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    M[:3, 3] = [x, y, 0.0]
    return M


class H(Node):
    def __init__(self):
        super().__init__('clamp_regress')
        self.cmd = np.zeros(3)
        self.cmds = []
        self.goal_base = None
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 100)
        self.create_subscription(PoseStamped, '/spacemouse/right/ee_target_pose', self.on_goal, 100)
        self.goal = self.create_publisher(PoseStamped, '/teleop_core/right/ee_goal', 10)
        self.sm = self.create_publisher(Twist, '/teleop_core/base/spacemouse', 10)
        self.ach = self.create_publisher(PoseStamped, '/ik_solver/achieved_ee_pose_r', 10)
        self.tfb = TransformBroadcaster(self)
        self.T_bm = T(1.2, 0.0, 0.0)
        self.sm_cmd = None
        self.lag = None          # fixed achieved pose (arm not following)
        self.last = time.time()

    def on_cmd(self, m):
        self.cmd = np.array([m.linear.x, m.linear.y, m.angular.z])
        self.cmds.append(self.cmd.copy())

    def on_goal(self, m):
        self.goal_base = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            now_t = time.time(); dt = now_t - self.last; self.last = now_t
            vx, vy, wz = self.cmd
            self.T_bm = np.linalg.inv(T(vx * dt, vy * dt, wz * dt)) @ self.T_bm   # base moves
            now = self.get_clock().now()
            t = TransformStamped(); t.header.stamp = now.to_msg()
            t.header.frame_id = 'base_link'; t.child_frame_id = 'marker_frame'
            t.transform.translation.x, t.transform.translation.y = self.T_bm[0, 3], self.T_bm[1, 3]
            yaw = math.atan2(self.T_bm[1, 0], self.T_bm[0, 0])
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.tfb.sendTransform(t)
            a = self.lag if self.lag is not None else self.goal_base
            if a is not None:
                p = PoseStamped(); p.header.stamp = now.to_msg(); p.header.frame_id = 'map'
                p.pose.position.x, p.pose.position.y, p.pose.position.z = a
                p.pose.orientation.w = 1.0
                self.ach.publish(p)
            if self.sm_cmd is not None:
                self.sm.publish(self.sm_cmd)
            rclpy.spin_once(self, timeout_sec=0.01)


def twist(vx=0.0, vy=0.0, wz=0.0):
    t = Twist(); t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
    return t


def main():
    rclpy.init()
    h = H()
    r = {}
    t0 = time.time()
    while time.time() - t0 < 10 and not (h.goal.get_subscription_count() and h.sm.get_subscription_count()):
        h.pump(0.1)
    h.pump(0.3)
    # goal at base (0.60, -0.30, 1.00) -> marker coordinates
    gb = np.array([0.60, -0.30, 1.00, 1.0])
    gm = np.linalg.inv(h.T_bm) @ gb
    g = PoseStamped(); g.header.frame_id = 'marker_frame'; g.pose.orientation.w = 1.0
    g.pose.position.x, g.pose.position.y, g.pose.position.z = gm[:3]
    h.goal.publish(g); h.pump(0.5)
    # drive backwards (away from the board): goal x grows toward the 0.75 limit
    h.sm_cmd = twist(vx=-0.5)
    h.pump(3.0)
    r['goal_x_after_reverse'] = round(float(h.goal_base[0]), 4)       # <= 0.75
    r['cmd_at_limit'] = [round(float(v), 4) for v in h.cmd]            # ~0
    h.cmds.clear()
    h.sm_cmd = twist(vx=0.5)                                           # forward is free
    h.pump(0.3)
    r['forward_passes'] = round(float(h.cmd[0]), 3)                    # 0.1 (cap)
    h.sm_cmd = twist()
    h.pump(0.5)
    # move well inside the box first (forward 1.5 s at the 0.1 cap)
    h.sm_cmd = twist(vx=0.5); h.pump(1.5)
    h.sm_cmd = twist(); h.pump(0.3)
    r['goal_x_inside'] = round(float(h.goal_base[0]), 3)
    # reactive: arm stuck 3 cm behind (-x of) the goal
    h.lag = h.goal_base + np.array([-0.03, 0.0, 0.0])
    h.cmds.clear()
    h.sm_cmd = twist(vx=-0.5)                    # moves goal +x: away from the lagging arm
    h.pump(0.4)
    r['tripped_away_max'] = round(float(max(abs(c[0]) for c in h.cmds)), 4) if h.cmds else None  # ~0 (first tick may pass)
    h.cmds.clear()
    h.sm_cmd = twist(vx=0.5)                     # moves goal -x: toward the arm
    h.pump(0.2)
    r['tripped_toward_max'] = round(float(max(c[0] for c in h.cmds)), 3) if h.cmds else None   # 0.1 passes
    h.sm_cmd = twist(); h.pump(0.3)
    h.lag = None                                 # arm catches up
    h.sm_cmd = twist(vx=-0.5); h.pump(0.3)
    r['released_cmd'] = round(float(h.cmd[0]), 3)                      # -0.1 (unless box limit)
    print(json.dumps(r, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
