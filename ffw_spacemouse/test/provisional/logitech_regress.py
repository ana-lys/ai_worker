#!/usr/bin/env python3
"""logitech_base_teleop (use_teleop_core:=true) -> teleop_core: no take-over,
twist merged behind the SpaceMouse, head from /joint_states, lift targets."""
import json, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


class H(Node):
    def __init__(self):
        super().__init__('logitech_regress')
        self.cmds, self.heads, self.goals = [], [], []
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.cmds.append((m.linear.x, m.linear.y)), 100)
        self.create_subscription(JointTrajectory, '/leader/joystick_controller_left/joint_trajectory',
                                 lambda m: self.heads.append((list(m.points[0].positions),
                                     m.points[0].time_from_start.sec + m.points[0].time_from_start.nanosec * 1e-9)), 100)
        self.create_subscription(PoseStamped, '/spacemouse/left/ee_target_pose',
                                 lambda m: self.goals.append(m.pose.position.z), 100)
        self.joy = self.create_publisher(Joy, '/joy_logitech', 10)
        self.mode = self.create_publisher(String, '/teleop_mode', 10)
        self.js = self.create_publisher(JointState, '/joint_states', 10)
        self.sm = self.create_publisher(Twist, '/teleop_core/base/spacemouse', 10)
        self.goal = self.create_publisher(PoseStamped, '/teleop_core/left/ee_goal', 10)
        self.axes, self.btn = [0.0] * 6, [0] * 12
        self.sm_cmd = None

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            j = Joy(); j.header.stamp = self.get_clock().now().to_msg()
            j.axes = list(self.axes); j.buttons = list(self.btn)
            self.joy.publish(j)
            s = JointState(); s.name = ['head_joint1', 'head_joint2']; s.position = [0.2, -0.1]
            self.js.publish(s)
            if self.sm_cmd is not None:
                self.sm.publish(self.sm_cmd)
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    rclpy.init()
    h = H()
    r = {}
    t0 = time.time()
    while time.time() - t0 < 10 and not (h.joy.get_subscription_count() and h.sm.get_subscription_count()
                                         and h.count_publishers('/teleop_core/base/logitech')):
        h.pump(0.1)
    for _ in range(3):
        h.mode.publish(String(data='BASE')); h.pump(0.1)
    # stick forward half: (0.5^2 - 0.04) * 1.04 * 0.6 = 0.131, no 2 s hold needed
    h.axes = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]; h.pump(0.4)
    r['cmd_x_no_takeover'] = round(h.cmds[-1][0], 3) if h.cmds else None     # 0.131
    # SpaceMouse moving wins
    tw = Twist(); tw.linear.y = 0.2; h.sm_cmd = tw; h.pump(0.3)
    r['sm_wins'] = [round(v, 3) for v in h.cmds[-1]]                          # [0, 0.2]
    h.sm_cmd = None; h.axes = [0.0] * 6; h.pump(0.6)
    # hat up: step from the measured head (0.2, -0.1), tracked duration
    h.heads.clear()
    h.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]; h.pump(0.1)
    h.axes = [0.0] * 6; h.pump(0.2)
    r['head_first_press'] = [round(v, 3) for v in h.heads[0][0]] if h.heads else None   # [0.15, -0.1]
    r['head_duration_s'] = round(h.heads[0][1], 3) if h.heads else None                 # 0.125
    # second press within 1 s steps from the commanded position
    h.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]; h.pump(0.1)
    h.axes = [0.0] * 6; h.pump(0.2)
    r['head_second_press'] = [round(v, 3) for v in h.heads[-1][0]] if h.heads else None  # [0.1, -0.1]
    # elevation button A (index 5): -0.10 m target on a base_link goal
    g = PoseStamped(); g.header.frame_id = 'base_link'; g.pose.orientation.w = 1.0
    g.pose.position.x, g.pose.position.y, g.pose.position.z = 0.4, 0.3, 1.0
    h.goal.publish(g); h.pump(0.3)
    h.btn[5] = 1; h.pump(0.1); h.btn[5] = 0; h.pump(1.5)
    r['lift_target_z'] = round(h.goals[-1], 3) if h.goals else None          # 0.9
    print(json.dumps(r, indent=1, sort_keys=True))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
