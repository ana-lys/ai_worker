"""Baseline robot-side ZMQ node -- run on the machine attached to the robot.

Publishes RobotState (PUB, latest-wins) on tcp://*:STATE_PORT.
Subscribes to ControlCmd (SUB) on tcp://*:CONTROL_PORT.

For the baseline, build_state() emits synthetic sine waves so the link works
with no hardware. Replace build_state() with real joint reads (libfranka /
franka_ros2 state callback) when you have them.

Usage:
    python robot_node.py [--state-port 6001] [--control-port 6002] [--hz 100]
"""

import argparse
import math
import os
import sys
import time

import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as proto

STATE_PORT = 6001
CONTROL_PORT = 6002


def build_state(now_s: float) -> proto.RobotState:
    """Synthetic joint data. Swap for real reads: 20 x (pos, vel, acc)."""
    st = proto.RobotState()
    for i in range(proto.N_JOINTS):
        st.joint_pos[i] = math.sin(now_s * 0.5 + i * 0.3)
        st.joint_vel[i] = 0.5 * math.cos(now_s * 0.5 + i * 0.3)
        st.joint_acc[i] = -0.25 * math.sin(now_s * 0.5 + i * 0.3)
    return st


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state-port", type=int, default=STATE_PORT)
    ap.add_argument("--control-port", type=int, default=CONTROL_PORT)
    ap.add_argument("--hz", type=float, default=100.0)
    args = ap.parse_args()

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.CONFLATE, 1)  # state is latest-wins telemetry
    pub.bind(f"tcp://*:{args.state_port}")
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.CONFLATE, 1)  # only the newest control target matters
    sub.bind(f"tcp://*:{args.control_port}")

    print(f"robot node: state-> {args.state_port} (PUB), control<- {args.control_port} (SUB), {args.hz:.0f} Hz")

    period = 1.0 / args.hz
    next_send = time.monotonic()
    last_print = 0.0
    while True:
        # Drain pending control commands (non-blocking); keep the newest.
        cmd = None
        while True:
            try:
                cmd, _ts = proto.decode_control(sub.recv(zmq.DONTWAIT))
            except zmq.Again:
                break
        if cmd is not None and time.monotonic() - last_print > 0.2:
            last_print = time.monotonic()
            e0, e1 = cmd.ee
            print(f"[ctl] ee0=({e0[0]:+.3f},{e0[1]:+.3f},{e0[2]:+.3f},"
                  f"{e0[3]:+.3f},{e0[4]:+.3f},{e0[5]:+.3f}) "
                  f"ee1=({e1[0]:+.3f},{e1[1]:+.3f},{e1[2]:+.3f},"
                  f"{e1[3]:+.3f},{e1[4]:+.3f},{e1[5]:+.3f})")

        now = time.monotonic()
        if now >= next_send:
            pub.send(proto.encode_robot_state(build_state(time.time())))
            next_send += period
        time.sleep(0.001)


if __name__ == "__main__":
    main()
