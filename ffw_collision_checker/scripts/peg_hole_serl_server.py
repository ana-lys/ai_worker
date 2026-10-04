#!/usr/bin/env python3
"""HIL-SERL peg-in-hole server for the REAL robot (ffw_peg_hole_env/SERL_REAL_PLAN.md).

Publishes one Frame per 15 Hz tick on base+0: the gateway's Obs (joint block in A,
achieved IK EE poses, grippers; marker block at defaults) plus the tag
(episode_id, step, frame_state, reason, reward, applied action), and listens on
base+1 for ControlCmdDelta / EnvCmd (ffw_peg_hole_env/wire.py).

Slice 2: IDLE frames only, no motion. Commands are counted and logged.

  source ROS; .venv/bin/python peg_hole_serl_server.py --port-base 7601
"""
import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
import zmq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "ffw_peg_hole_env"))
sys.path.insert(0, str(HERE.parents[1] / "ffw_zmqinterface"))
from ffw_peg_hole_env import wire  # noqa: E402
from ffw_zmqinterface import gateway_node as gwn, protocol as gw  # noqa: E402
import peg_hole_teach as pht  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

HZ = 15.0


def obs_frame(io):
    """Gateway-identical Obs frame from /joint_states + /ik_solver/achieved_ee_pose_{r,l},
    or None until both are in."""
    raw = io.raw_js
    sites = [io.latest_site("right"), io.latest_site("left")]          # Obs ee0 = right, ee1 = left
    if raw is None or any(s is None for s in sites):
        return None
    js = type("JS", (), {"name": raw[1], "position": raw[2], "velocity": raw[3], "effort": raw[4]})
    o = gw.Obs()
    o.joint_pos, o.joint_vel, o.joint_effort = gwn._reorder_joint_block(js)      # effort -> A for the arms
    ee = []
    for T in sites:
        x, y, z, w = Rot.from_matrix(T[:3, :3]).as_quat()
        ee.append((*T[:3, 3], *gw.quat_to_rpy(w, x, y, z)))
    o.ee = (tuple(float(v) for v in ee[0]), tuple(float(v) for v in ee[1]))
    pos = dict(zip(raw[1], raw[2]))
    o.gripper = tuple(gwn._normalize_grip(pos.get(gwn._GRIP_JOINT[i]), i) for i in (0, 1))
    return gw.encode_obs(o)


class SerlServer:
    def __init__(self, io, port_base, host="*"):
        self.io = io
        ctx = zmq.Context.instance()
        self.pub = ctx.socket(zmq.PUB)
        self.pub.bind(f"tcp://{host}:{port_base + wire.PORT_OBS}")
        self.sub = ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.sub.bind(f"tcp://{host}:{port_base + wire.PORT_CONTROL}")
        self.episode_id, self.step = 0, 0
        self.state, self.reason, self.reward, self.action = wire.FS_IDLE, wire.TR_NONE, 0.0, np.zeros(6)
        self.counts = {"delta": 0, "env_cmd": 0, "bad": 0, "frames": 0}

    def poll_commands(self):
        """Drain the control socket -> (latest right-arm delta or None, [EnvCmd dicts])."""
        delta, cmds = None, []
        while self.sub.poll(0):
            data = self.sub.recv()
            try:
                t = wire.msg_type(data)
                if t == wire.MSG_CONTROL_DELTA:
                    delta = np.array(wire.decode_delta(data)[0][:6])
                    self.counts["delta"] += 1
                elif t == wire.MSG_ENV_CMD:
                    cmds.append(wire.decode_env_cmd(data)[0])
                    self.counts["env_cmd"] += 1
                else:
                    self.counts["bad"] += 1
            except (ValueError, Exception):
                self.counts["bad"] += 1
        return delta, cmds

    def publish(self):
        f = obs_frame(self.io)
        if f is None:
            return False
        self.pub.send(wire.encode_frame(f, self.episode_id, self.step, self.state, self.reason,
                                        self.reward, self.action))
        self.counts["frames"] += 1
        return True

    def run(self, duration=None):
        period, t_next, t0, last_log = 1.0 / HZ, time.monotonic(), time.monotonic(), 0.0
        while duration is None or time.monotonic() - t0 < duration:
            delta, cmds = self.poll_commands()
            for c in cmds:
                print(f"EnvCmd {wire.CMD_NAMES.get(c['cmd'], c['cmd'])} seed {c['seed']} episode {c['episode_id']} "
                      f"(slice 2: ignored, frames stay IDLE)")
            self.publish()
            now = time.monotonic()
            if now - last_log > 5.0:
                print(f"  {self.counts}")
                last_log = now
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()))
            if time.monotonic() - t_next > period:                    # fell behind: don't burst
                t_next = time.monotonic()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port-base", type=int, default=7601)
    ap.add_argument("--duration", type=float, default=None, help="s, then quit (tests)")
    a = ap.parse_args()
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    io = pht.TeachIo(False)                          # slice 2: subscribe only, never commands
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        t0 = time.monotonic()
        while obs_frame(io) is None and time.monotonic() - t0 < 15.0:
            time.sleep(0.1)
        if obs_frame(io) is None:
            print("no /joint_states or achieved EE poses in 15 s - is the teleop running?")
            return
        srv = SerlServer(io, a.port_base)
        print(f"serving IDLE frames at {HZ:.0f} Hz on {a.port_base} (+{wire.PORT_OBS} frames, +{wire.PORT_CONTROL} control)")
        srv.run(a.duration)
        print(f"done: {srv.counts}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        ex.shutdown()
        spin.join(timeout=2.0)
        io.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
