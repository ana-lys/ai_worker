#!/usr/bin/env python3
"""joy_base (use_teleop_core:=true, hardware_mode:=false) -> teleop_core:
base twist, left-mouse lift and head all arrive through teleop_core."""
import json, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory


class H(Node):
    def __init__(self):
        super().__init__('joy_base_regress')
        self.cmds, self.heads, self.modes, self.goals = [], [], [], []
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.cmds.append((m.linear.x, m.angular.z)), 100)
        self.create_subscription(JointTrajectory, '/leader/joystick_controller_left/joint_trajectory',
                                 lambda m: self.heads.append(list(m.points[0].positions)), 100)
        self.create_subscription(String, '/teleop_mode', lambda m: self.modes.append(m.data), 10)
        self.create_subscription(PoseStamped, '/spacemouse/left/ee_target_pose',
                                 lambda m: self.goals.append((time.time(), m.pose.position.z)), 100)
        self.rj = self.create_publisher(Joy, '/right/joy', 10)
        self.lj = self.create_publisher(Joy, '/left/joy', 10)
        self.goal = self.create_publisher(PoseStamped, '/teleop_core/left/ee_goal', 10)
        self.latch = self.create_publisher(Bool, '/logitech/base_active', 10)
        self.r_axes, self.l_axes = [0.0] * 6, [0.0] * 6
        self.r_btn, self.l_btn = [0, 0], [0, 0]

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            for pub, ax, b in ((self.rj, self.r_axes, self.r_btn), (self.lj, self.l_axes, self.l_btn)):
                j = Joy(); j.header.stamp = self.get_clock().now().to_msg()
                j.axes = list(ax); j.buttons = list(b)
                pub.publish(j)
            rclpy.spin_once(self, timeout_sec=0.01)

    def double_click_all(self):
        for _ in range(2):
            self.r_btn = self.l_btn = [1, 1]; self.pump(0.06)
            self.r_btn = self.l_btn = [0, 0]; self.pump(0.06)


def main():
    rclpy.init()
    h = H()
    r = {}
    t0 = time.time()
    while time.time() - t0 < 10 and not (h.rj.get_subscription_count() and h.goal.get_subscription_count()
                                         and h.count_publishers('/cmd_vel')):
        h.pump(0.1)
    h.pump(1.0)
    h.double_click_all(); h.pump(0.3)
    r['mode'] = h.modes[-1] if h.modes else None                      # BASE
    # base twist through teleop_core (cubic: 0.5^3 * max_linear 1.0)
    h.r_axes = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]; h.pump(0.4)
    r['cmd_x'] = round(h.cmds[-1][0], 4) if h.cmds else None          # 0.125
    # the old Logitech latch no longer silences joy_base
    h.latch.publish(Bool(data=True)); h.pump(0.4)
    r['cmd_x_with_latch'] = round(h.cmds[-1][0], 4) if h.cmds else None   # 0.125
    h.r_axes = [0.0] * 6; h.pump(0.5)
    # left-mouse lift: a base_link goal rides up while z is held, stops on release
    g = PoseStamped(); g.header.frame_id = 'base_link'; g.pose.orientation.w = 1.0
    g.pose.position.x, g.pose.position.y, g.pose.position.z = 0.4, 0.3, 1.0
    h.goal.publish(g); h.pump(0.3); h.goals.clear()
    h.l_axes = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]; h.pump(1.0)
    t_rel = time.time()
    h.l_axes = [0.0] * 6; h.pump(0.6)
    z = [x[1] for x in h.goals]
    r['lift_1s_m'] = round(z[-1] - 1.0, 2) if z else None             # ~0.1 (0.1 m/s)
    after = [x[1] for x in h.goals if x[0] > t_rel + 0.1]
    # teleop_core republishes a base_link goal only when it changes
    r['lift_stops_on_release'] = (not after) or (max(after) - min(after) < 0.002)
    # head through teleop_core
    h.heads.clear()
    h.l_axes = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0]; h.pump(0.3)
    h.l_axes = [0.0] * 6; h.pump(0.2)
    r['head_relayed'] = len(h.heads) > 0
    print(json.dumps(r, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
