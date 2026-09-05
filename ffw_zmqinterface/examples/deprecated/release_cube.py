#!/usr/bin/env python3
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
"""release_cube.py — open the RIGHT gripper to drop the cube, holding both arms.

Live robot command via the framed ZMQ gateway (the same path a controller uses).
Right arm holds the cube (grip0 ~ 0.68); we open it (grip0 -> 0.0) so the cube
drops. Both arms are held at their current achieved poses, so the solver leash
sees zero error and neither arm moves.

Requires the joy_hand TRACK fix: while /control_override is TRUE, joy_hand
publishes "TRACK" on /quest/<arm>/state, so the solver's update_grippers()
obeys the gateway's trigger feed. The script verifies TRACK on the state topic
before sending the open command; if the state is not TRACK it aborts without
touching the grippers (gripper path is TRACK-gated — a CTRL state would ignore
the trigger, exactly like the earlier failed release attempt).

Run (from the ffw_zmqinterface package root):
    ROS_DOMAIN_ID=30 python3 examples/release_cube.py
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, String

import zmq

# Use the installed package when the workspace is sourced, else the source tree.
try:
    import ffw_zmqinterface.protocol as proto
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "ffw_zmqinterface"))
    import protocol as proto

GW_HOST = "192.168.0.249"  # gateway host (set to 127.0.0.1 for a local stack)
GW_CONTROL = f"tcp://{GW_HOST}:6002"
GW_STATE = f"tcp://{GW_HOST}:6001"
OVERRIDE_TOPIC = "/control_override"
STATE_TOPIC = "/quest/right/state"
TRIGGER_TOPIC = "/quest/right/trigger"
RELEASE_S = 3.0       # how long to hold the gripper open
SEND_HZ = 25.0
TRACK_WAIT_S = 0.5    # give joy_hand time to publish TRACK after override
STARTUP_S = 0.3       # let the Obs stream warm up before sampling grip0

achieved = {"r": None, "l": None}
grip_fb = [None, None]   # Obs gripper feedback from ZMQ 6001
state_seen = []          # /quest/right/state echoes (expect "TRACK")
trigger_echo = []        # /quest/right/trigger echoes (expect 0.0)
lock = threading.Lock()


class Probe(Node):
    def __init__(self):
        super().__init__("gw_release_probe")
        self.create_subscription(PoseStamped, "/ik_solver/achieved_ee_pose_r", self._on_r, 10)
        self.create_subscription(PoseStamped, "/ik_solver/achieved_ee_pose_l", self._on_l, 10)
        self.create_subscription(String, STATE_TOPIC, self._on_state, 10)
        self.create_subscription(Float32, TRIGGER_TOPIC, self._on_trigger, 10)
        self.ovr_pub = self.create_publisher(Bool, OVERRIDE_TOPIC, 10)

    def _on_r(self, m):
        with lock:
            achieved["r"] = m

    def _on_l(self, m):
        with lock:
            achieved["l"] = m

    def _on_state(self, m):
        with lock:
            state_seen.append(m.data)

    def _on_trigger(self, m):
        with lock:
            trigger_echo.append(m.data)

    def set_override(self, value, pulses=5):
        msg = Bool()
        msg.data = value
        for _ in range(pulses):
            self.ovr_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        print(f"  /control_override -> {value} ({pulses} pulses)", flush=True)


def zmq_state_loop(sub, stop):
    """Drain the gateway's obs stream (6001), keeping the newest Obs grippers."""
    while not stop.is_set():
        try:
            frames = sub.recv_multipart()
        except zmq.ZMQError:
            break
        for f in frames:
            st, _ = proto.decode_obs(f)   # 6001 carries Obs only (v2)
            with lock:
                grip_fb[0] = st.gripper[0]
                grip_fb[1] = st.gripper[1]


def xyzrpy(m):
    p = m.pose
    rx, ry, rz = proto.quat_to_rpy(p.orientation.w, p.orientation.x,
                                   p.orientation.y, p.orientation.z)
    return (p.position.x, p.position.y, p.position.z, rx, ry, rz)


def main():
    rclpy.init()
    node = Probe()
    stop = threading.Event()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect(GW_STATE)
    thread = threading.Thread(target=zmq_state_loop, args=(sub, stop), daemon=True)
    thread.start()

    pub = ctx.socket(zmq.PUB)
    pub.connect(GW_CONTROL)
    time.sleep(0.3)  # SUB/PUB handshake

    try:
        # --- wait for live achieved poses + gripper feedback ---------------
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            with lock:
                if achieved["r"] is not None and achieved["l"] is not None:
                    break
        else:
            print("FAIL: no achieved poses — is the stack up?", flush=True)
            sys.exit(1)

        time.sleep(STARTUP_S)
        with lock:
            grip0_start = grip_fb[0]
            grip1_start = grip_fb[1]
            start_r = xyzrpy(achieved["r"])
            start_l = xyzrpy(achieved["l"])

        if grip0_start is None or grip0_start < 0.3:
            print(f"FAIL: grip0 feedback is {grip0_start} — right gripper is not "
                  f"closed (no cube?). Refusing to run.", flush=True)
            sys.exit(2)

        print(f"RIGHT gripper start: grip0 = {grip0_start:.3f}  (cube held)", flush=True)
        print(f"LEFT  gripper held at grip1 = {grip1_start:.3f}", flush=True)
        print(f"RIGHT arm @ ({start_r[0]:.3f},{start_r[1]:.3f},{start_r[2]:.3f}) "
              f"rpy ({start_r[3]:.2f},{start_r[4]:.2f},{start_r[5]:.2f})", flush=True)
        print(f"LEFT  arm @ ({start_l[0]:.3f},{start_l[1]:.3f},{start_l[2]:.3f})", flush=True)

        # --- engage override (joy_hand now forces TRACK) -------------------
        print("\n*** WARNING: cube will drop. Hands OFF the SpaceMouse, clear the "
              "drop area. ***", flush=True)
        print("STEP 1: enabling override ...", flush=True)
        node.set_override(True)

        t_track = time.monotonic() + TRACK_WAIT_S
        while time.monotonic() < t_track:
            rclpy.spin_once(node, timeout_sec=0.02)
        with lock:
            states = list(state_seen)
        if not states or "TRACK" not in states:
            print(f"FAIL: /quest/right/state shows {states or 'nothing'} — TRACK fix "
                  f"not live? Releasing override without touching the gripper.",
                  flush=True)
            node.set_override(False)
            sys.exit(3)
        print(f"  /quest/right/state -> {states} (TRACK confirmed, gripper feed live)", flush=True)

        # --- open the right gripper while holding both arms -----------------
        print(f"STEP 2: opening right gripper for {RELEASE_S:.0f}s @ {SEND_HZ:.0f}Hz ...", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < RELEASE_S:
            with lock:
                r = achieved["r"]
                l = achieved["l"]
                grip1_now = grip_fb[1] if grip_fb[1] is not None else 0.0
            if r is not None and l is not None:
                goal = (xyzrpy(r), xyzrpy(l))
                pub.send(proto.encode_control(
                    proto.ControlCmd(goal, gripper=(0.0, grip1_now)), ts=time.time()))
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(1.0 / SEND_HZ)

        time.sleep(0.2)  # let the last frame settle + feedback update
        with lock:
            grip0_end = grip_fb[0]
        print(f"STEP 3: releasing override ...", flush=True)
        node.set_override(False)

        # --- report ---------------------------------------------------------
        with lock:
            n_trig = len(trigger_echo)
            trig_ok = all(abs(v) < 0.01 for v in trigger_echo) if trigger_echo else False
        print(f"\nRIGHT gripper: {grip0_start:.3f} -> {grip0_end:.3f}", flush=True)
        print(f"trigger echoes: {n_trig} frames, all ~0.0 = {trig_ok}", flush=True)
        if grip0_end is not None and grip0_end < max(0.2, grip0_start - 0.3):
            print("RESULT: PASS — right gripper opened, cube released.", flush=True)
            sys.exit(0)
        print(f"RESULT: FAIL — grip0 ended at {grip0_end} (was {grip0_start}). "
              f"Gripper may not have moved.", flush=True)
        sys.exit(4)
    finally:
        stop.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
