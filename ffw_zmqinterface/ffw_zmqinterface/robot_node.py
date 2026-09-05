"""Baseline robot-side ZMQ node -- run on the machine attached to the robot.

Publishes Obs (PUB) on tcp://*:STATE_PORT  (6001) -- one complete, self-
contained observation per tick: the full joint block in JOINT_STATE_NAMES order
(mirroring a recorded state.csv row) with the dual-EE block at the tail, exactly
as gateway_node.py emits it (protocol.py).
Subscribes to ControlCmd (SUB) on tcp://*:CONTROL_PORT (6002).

For the baseline, build_obs() emits synthetic sine waves over the joint block
so the link works with no hardware (the EE block stays zeros = a pose the
controller should not follow). Replace build_obs() with real joint reads
(libfranka / franka_ros2 state callback) when you have them -- and note the
Obs cadence contract: every tick, complete observations, no gaps between the
EE block and the joint block (the real gateway repeats the newest joint block
between /joint_states broadcasts).

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


def build_obs(now_s: float) -> proto.Obs:
    """Synthetic joint data. Swap for real reads: N_JOINT_STATE joints x3.

    The EE block stays zeros (grippers open) -- a synthetic pose the
    controller must not follow. The joint block is the interesting part:
    pos/vel/effort, all N_JOINT_STATE in JOINT_STATE_NAMES order.
    """
    obs = proto.Obs()  # EE zeros, grippers open (0.0)
    for i in range(proto.N_JOINT_STATE):
        obs.joint_pos[i] = math.sin(now_s * 0.5 + i * 0.3)
        obs.joint_vel[i] = 0.5 * math.cos(now_s * 0.5 + i * 0.3)
        obs.joint_effort[i] = -0.25 * math.sin(now_s * 0.5 + i * 0.3)
    return obs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state-port", type=int, default=STATE_PORT)
    ap.add_argument("--control-port", type=int, default=CONTROL_PORT)
    ap.add_argument("--hz", type=float, default=100.0)
    args = ap.parse_args()

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{args.state_port}")
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.CONFLATE, 1)  # only the newest control target matters
    sub.bind(f"tcp://*:{args.control_port}")

    print(f"robot node: obs-> {args.state_port} (PUB), control<- {args.control_port} (SUB), {args.hz:.0f} Hz")

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
            # No CONFLATE on the Obs PUB: every tick is a complete observation
            # for sync-and-concat consumers, so each frame must reach them.
            pub.send(proto.encode_obs(build_obs(time.time())))
            next_send += period
        time.sleep(0.001)


if __name__ == "__main__":
    main()
