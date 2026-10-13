#!/usr/bin/env python3
"""End to end, sim: connector_align_capture.py against the plain unified launch
(hardware_mode:=false, real IK solver). This harness is the robot's world: it
integrates /cmd_vel into the base pose and publishes /odom, the ArUco board pose
(/head_camera/marker_board_pose), /marker_ekf_odom and /joint_states from it.
It answers every Enter prompt and checks, from the TRUE board pose, that the
right EE stands at the stand-off in front of each point and that each base
placement lands where it was planned."""
import json, math, os, re, subprocess, sys, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

SCRIPTS = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_spacemouse/scripts')
sys.path.insert(0, SCRIPTS)
from connector_align_capture import load_targets, Rz  # noqa: E402

TARGETS = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_spacemouse/data/pose_snapshots.json')
OFFSET = 0.0254
STANDOFF = 0.015
# true board in the START base frame (level): the robot's 2nd real placement
BOARD_XY = np.array([0.8216, 0.0684])
BOARD_Z = 0.7596
BOARD_YAW = math.radians(-98.78)


def T2(x, y, yaw):
    M = np.eye(3); M[:2, :2] = Rz(yaw)[:2, :2]; M[:2, 2] = [x, y]
    return M


class World(Node):
    def __init__(self):
        super().__init__('e2e_connector_world')
        self.base = np.array([0.0, 0.0, 0.0])     # world (= start base) x y yaw
        self.cmd = np.zeros(3)
        self.ach = None
        self.goal = None
        self.create_subscription(PoseStamped, '/quest/right/ee_target_pose',
                                 lambda m: setattr(self, 'goal', np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])), 20)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', np.array([m.linear.x, m.linear.y, m.angular.z])), 20)
        self.create_subscription(PoseStamped, '/ik_solver/achieved_ee_pose_r',
                                 lambda m: setattr(self, 'ach', np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])), 20)
        self.odom = self.create_publisher(Odometry, '/odom', 20)
        self.det = self.create_publisher(PoseStamped, '/head_camera/marker_board_pose', qos_profile_sensor_data)
        self.ekf = self.create_publisher(Odometry, '/marker_ekf_odom', 20)
        self.js = self.create_publisher(JointState, '/joint_states', 20)
        self.k = 0
        self.t = time.monotonic()
        self.create_timer(0.02, self.step)

    def board_in_base(self):
        Tb = np.linalg.inv(T2(*self.base)) @ T2(BOARD_XY[0], BOARD_XY[1], BOARD_YAW)
        return Tb[:2, 2], math.atan2(Tb[1, 0], Tb[0, 0])

    def step(self):
        now = time.monotonic(); dt = now - self.t; self.t = now
        vx, vy, wz = self.cmd
        c, s = math.cos(self.base[2]), math.sin(self.base[2])
        self.base += [(c * vx - s * vy) * dt, (s * vx + c * vy) * dt, wz * dt]
        self.k += 1
        st = self.get_clock().now().to_msg()
        o = Odometry(); o.header.stamp = st; o.header.frame_id = 'odom'
        o.pose.pose.position.x, o.pose.pose.position.y = self.base[:2]
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.base[2] / 2), math.cos(self.base[2] / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, wz
        self.odom.publish(o)
        if self.k % 5 == 0:   # 10 Hz detections
            p, yaw = self.board_in_base()
            d = PoseStamped(); d.header.stamp = st; d.header.frame_id = 'base_link'
            d.pose.position.x, d.pose.position.y, d.pose.position.z = p[0], p[1], BOARD_Z
            d.pose.orientation.z, d.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.det.publish(d)
            Tm = np.linalg.inv(T2(BOARD_XY[0], BOARD_XY[1], BOARD_YAW)) @ T2(*self.base)
            e = Odometry(); e.header.stamp = st; e.header.frame_id = 'marker_frame'
            e.pose.pose.position.x, e.pose.pose.position.y = Tm[0, 2], Tm[1, 2]
            ye = math.atan2(Tm[1, 0], Tm[0, 0])
            e.pose.pose.orientation.z, e.pose.pose.orientation.w = math.sin(ye / 2), math.cos(ye / 2)
            self.ekf.publish(e)
        j = JointState(); j.header.stamp = st; j.name = ['head_joint1', 'head_joint2']; j.position = [0.0, 0.0]
        self.js.publish(j)


def main():
    rclpy.init()
    w = World()
    threading.Thread(target=lambda: rclpy.spin(w), daemon=True).start()
    targets = load_targets(TARGETS, OFFSET)
    out = f'/tmp/ffw_teleop_provisional/e2e_connector_align_{int(time.time())}.json'
    proc = subprocess.Popen(['ros2', 'run', 'ffw_spacemouse', 'connector_align_capture.py', '--placements', '2',
                             '--seed', '3', '--out', out,
                             # sim speed: fast arm/base, no settle wait (the sim solver stops short anyway)
                             '--arm-vmax', '0.3', '--arm-wmax-deg', '90', '--settle-timeout', '0.2',
                             '--base-vmax', '0.3', '--base-wmax', '0.5', '--window', '0.4'], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=0)
    res = {'points': [], 'placements': []}
    buf = ''
    planned = []
    t_end = time.time() + 400
    while time.time() < t_end:
        ch = proc.stdout.read(1)
        if ch == '':
            break
        buf += ch
        if ch == '\n':
            line = buf.strip(); buf = ''
            m = re.match(r'B(\d): ([+-][\d.]+) ([+-][\d.]+) ([+-][\d.]+)$', line)
            if m:
                planned.append([float(m.group(2)), float(m.group(3)), math.radians(float(m.group(4)))])
            if 'SAFETY' in line:
                proc.stdin.write('\n')
            if line.startswith('done:') or line.startswith('ABORT') or 'NOT recorded' in line:
                res.setdefault('log', []).append(line)
            continue
        if buf.endswith('then Enter '):
            name = re.search(r'\[(\w+)\]', buf).group(1)
            time.sleep(0.5)                       # "align" (sim arm is already still)
            bi = int(name[1]) - 1
            p_board, yaw_b = w.board_in_base()
            pl, _, _ = targets[name[-1]]
            mx = np.array([math.cos(yaw_b + math.pi / 2), math.sin(yaw_b + math.pi / 2), 0.0])  # marker_frame x
            exp = np.append(Rz(yaw_b)[:2, :2] @ pl[:2] + p_board, pl[2]) + [OFFSET, 0, 0] - STANDOFF * mx
            err = (w.ach - exp) * 1000 if w.ach is not None else None
            gerr = (w.goal - exp) * 1000 if w.goal is not None else None
            res['points'].append((name, [round(float(v), 1) for v in err], [round(float(v), 1) for v in gerr]))
            if name.endswith('_ad_a'):
                res['placements'].append((name[:2], [round(float(v), 3) for v in w.base[:2]] + [round(math.degrees(w.base[2]), 2)],
                                          [round(v, 3) for v in planned[bi][:2]] + [round(math.degrees(planned[bi][2]), 2)] if bi < len(planned) else None))
            proc.stdin.write('\n')
            buf = ''
    proc.wait(timeout=20)
    errs = np.array([e for _, e, _ in res['points']])
    gerrs = np.array([g for _, _, g in res['points']])
    summary = {
        'recorded': len(json.load(open(out))) if os.path.exists(out) else 0,
        'prompts': len(res['points']),
        'achieved_vs_expected_mm_max': round(float(np.abs(errs).max()), 1) if len(errs) else None,
        'commanded_goal_vs_expected_mm_max': round(float(np.abs(gerrs).max()), 1) if len(gerrs) else None,
        'placement_true_vs_planned': res['placements'],
        'log': res.get('log'),
    }
    print(json.dumps(summary, indent=1))
    print('per point (name, achieved-expected mm, commanded goal-expected mm):', res['points'])
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
