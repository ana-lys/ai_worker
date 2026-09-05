# =============================================================================
# DEPRECATED (v2) -- the /control_override Bool latch is gone.
#
# v1: a client pulsed std_msgs/Bool on /control_override to force joy_hand
# into TRACK so the solver's gripper/goal feed from the gateway was obeyed.
# v2: that latch no longer exists. OverrideCmd is now the 25-qpos joint-space
# command rail (ffw_zmqinterface/README.md section 2.6) -- no 0/1 engage bit,
# and the gateway does not republish any Bool. joy_hand's force-TRACK coupling
# to /control_override is retired with it.
#
# This script sent EE-space ControlCmd over ZMQ and used the Bool only as an
# authority gate, so it can no longer work against the current stack. It is
# kept frozen in examples/deprecated/ as a historical record of the EE-space
# pattern. Do NOT run it on a live link, and do NOT re-add /control_override
# to bring it back -- re-derive the authority gate from the joint rail.
# =============================================================================
"""Right-EE rectangle sweep: x 0.45..0.60, y 0..-0.30, 10 loops.

Live robot command via the framed ZMQ gateway (the same path a controller uses).
Assumes the stack is up on ROS_DOMAIN_ID=30 with the gateway reachable on the
GW_HOST destination below (default 192.168.0.249:6002).

Rectangle corners (map frame, right arm, clockwise):
    A = (0.45, -0.03)
    B = (0.60, -0.03)
    C = (0.60, -0.30)
    D = (0.45, -0.30)
z, roll, pitch, yaw are held at the start pose. Left arm is locked at its start
pose (its goal re-asserted as its current achieved pose -> leash sees zero error).

Every ControlCmd carries a fresh header timestamp, so the gateway forwards each
re-assertion and the solver's per-message leash walks the arm to the current
corner; once within CORNER_TOL, we advance to the next.

/control_override is ON for the whole run (quest goals authoritative, spacemouse
silenced at the source) and released at the end. Keep hands OFF the SpaceMouse.

Run (from the ffw_zmqinterface package root):
    ROS_DOMAIN_ID=30 python3 examples/rectangle_sweep.py
"""
import csv
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import zmq

# Use the installed package when the workspace is sourced, else the source tree.
try:
    import ffw_zmqinterface.protocol as proto
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "ffw_zmqinterface"))
    import protocol as proto

X_MIN, X_MAX = 0.45, 0.60
Y_MIN, Y_MAX = -0.30, -0.03
CORNERS = [(X_MIN, Y_MAX), (X_MAX, Y_MAX), (X_MAX, Y_MIN), (X_MIN, Y_MIN)]  # A B C D
LOOPS = 10
CORNER_TOL = 0.015      # m; both x and y must be within this of the corner
CORNER_TIMEOUT = 10.0   # s per corner before logging a miss and moving on
SETTLE_S = 0.5
SEND_HZ = 25.0
GW_HOST = "192.168.0.249"  # gateway host (set to 127.0.0.1 for a local stack)
GW_ENDPOINT = f"tcp://{GW_HOST}:6002"
OVERRIDE_TOPIC = "/control_override"

achieved = {"r": None, "l": None}
samples = []            # (t, x, y) achieved right samples for the CSV
lock = threading.Lock()


class Probe(Node):
    def __init__(self):
        super().__init__("gw_rectangle_probe")
        self.create_subscription(PoseStamped, "/ik_solver/achieved_ee_pose_r", self._on_r, 10)
        self.create_subscription(PoseStamped, "/ik_solver/achieved_ee_pose_l", self._on_l, 10)
        self.ovr_pub = self.create_publisher(Bool, OVERRIDE_TOPIC, 10)

    def _on_r(self, m):
        with lock:
            achieved["r"] = m

    def _on_l(self, m):
        with lock:
            achieved["l"] = m

    def set_override(self, value, pulses=5):
        msg = Bool()
        msg.data = value
        for _ in range(pulses):
            self.ovr_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        print(f"  /control_override -> {value} ({pulses} pulses)", flush=True)


def xyzrpy(m):
    p = m.pose
    rx, ry, rz = proto.quat_to_rpy(p.orientation.w, p.orientation.x,
                                   p.orientation.y, p.orientation.z)
    return (p.position.x, p.position.y, p.position.z, rx, ry, rz)


def corner_goal(c, z0, ori):
    return (c[0], c[1], z0, ori[0], ori[1], ori[2])


def main():
    rclpy.init()
    node = Probe()
    t0_all = time.monotonic()
    try:
        # --- wait for live achieved poses ------------------------------------
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            with lock:
                if achieved["r"] is not None and achieved["l"] is not None:
                    break
        else:
            print("FAIL: no achieved poses — is the stack up?", flush=True)
            sys.exit(1)

        with lock:
            start_r = xyzrpy(achieved["r"])
            start_l = xyzrpy(achieved["l"])
        z0 = 0.78                        # hold z fixed at 0.78 (user request)
        ori = (start_r[3], start_r[4], start_r[5])
        print(f"start RIGHT: x={start_r[0]:.4f} y={start_r[1]:.4f} z={start_r[2]:.4f}", flush=True)
        print(f"goal z held at {z0:.3f}", flush=True)
        print(f"rectangle x [{X_MIN:.3f},{X_MAX:.3f}]  y [{Y_MIN:.3f},{Y_MAX:.3f}]  "
              f"{LOOPS} loops", flush=True)

        # --- override ON ------------------------------------------------------
        print("STEP 1: enabling override (hands OFF the SpaceMouse) ...", flush=True)
        node.set_override(True)

        # --- rectangle sweep --------------------------------------------------
        print(f"STEP 2: sweeping {LOOPS} loops @ {SEND_HZ:.0f}Hz ...", flush=True)
        ctx = zmq.Context()
        pub = ctx.socket(zmq.PUB)
        pub.setsockopt(zmq.CONFLATE, 1)
        pub.connect(GW_ENDPOINT)
        time.sleep(0.3)  # SUB/PUB handshake

        left_goal = (start_l[0], start_l[1], start_l[2], start_l[3], start_l[4], start_l[5])
        edge_times = []
        missed = 0
        for loop in range(LOOPS):
            for ci, c in enumerate(CORNERS):
                goal_r = corner_goal(c, z0, ori)
                t0 = time.monotonic()
                reached = False
                while time.monotonic() - t0 < CORNER_TIMEOUT:
                    pub.send(proto.encode_control(
                        proto.ControlCmd((goal_r, left_goal)), ts=time.time()))
                    rclpy.spin_once(node, timeout_sec=0.01)
                    with lock:
                        r = achieved["r"]
                    if r is not None:
                        rp = xyzrpy(r)
                        with lock:
                            samples.append((time.monotonic() - t0_all, rp[0], rp[1]))
                        if (abs(rp[0] - c[0]) < CORNER_TOL and
                                abs(rp[1] - c[1]) < CORNER_TOL):
                            reached = True
                            break
                    time.sleep(1.0 / SEND_HZ)
                dt = time.monotonic() - t0
                edge_times.append(dt)
                name = "ABCD"[ci]
                print(f"  loop {loop + 1:2d} corner {name} ({c[0]:.3f},{c[1]:.3f}): "
                      f"{'reached in ' + f'{dt:.2f}s' if reached else 'MISS ' + f'{dt:.1f}s'}",
                      flush=True)

        # --- settle, release --------------------------------------------------
        print(f"STEP 3: settling {SETTLE_S:.1f}s, releasing override ...", flush=True)
        t_end = time.monotonic() + SETTLE_S
        while time.monotonic() < t_end:
            pub.send(proto.encode_control(
                proto.ControlCmd((corner_goal(CORNERS[0], z0, ori), left_goal)),
                ts=time.time()))
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(1.0 / SEND_HZ)
        node.set_override(False)

        # --- report -----------------------------------------------------------
        total = time.monotonic() - t0_all
        with lock:
            final = xyzrpy(achieved["r"])
        print(f"final RIGHT: x={final[0]:.4f} y={final[1]:.4f} z={final[2]:.4f}", flush=True)
        n = len(edge_times)
        print(f"edges: {n} total, {missed} missed; total {total:.1f}s; "
              f"mean edge {sum(edge_times) / n:.2f}s", flush=True)
        csv_path = "/tmp/rectangle_path.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y"])
            w.writerows(samples)
        print(f"path samples ({len(samples)}) -> {csv_path}", flush=True)
        print("RESULT:", "PASS - all corners reached" if missed == 0 else
              f"FAIL - {missed} corner(s) not reached", flush=True)
        sys.exit(0 if missed == 0 else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
