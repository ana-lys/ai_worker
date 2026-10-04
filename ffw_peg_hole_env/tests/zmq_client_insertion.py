#!/usr/bin/env python3
"""End-to-end check over ZMQ, as HIL-SERL would drive it: start the server
(subprocess), then per episode send EnvCmd RESET, wait for READY, and stream
ControlCmdDelta computed only from the Obs EE poses (+ the fixed EE->tool
transforms of the model) until EnvStatus says terminated/truncated.

  ../ffw_collision_checker/scripts/.venv/bin/python tests/zmq_client_insertion.py --episodes 20
  ... --realtime        # server ticks at 15 Hz wall clock instead of lockstep
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import zmq
from scipy.spatial.transform import Rotation as Rot

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ffw_peg_hole_env import wire  # noqa: E402
from ffw_peg_hole_env.geometry import T_from, apply_delta, clip_delta, delta_between, pose_error  # noqa: E402
from ffw_peg_hole_env.peg_hole import PegHoleConfig, rel_aligned  # noqa: E402
from ffw_peg_hole_env.sim_backend import MujocoBackend  # noqa: E402


def ee_from_obs(o, i):
    x, y, z, rx, ry, rz = o.ee[i]
    return T_from([x, y, z], Rot.from_euler("XYZ", [rx, ry, rz]).as_matrix())   # R = Rx Ry Rz


class ObsSolver:
    """Same two-phase insertion as InsertionSolver, but from Obs only."""

    def __init__(self, ee_to_tool, tip, rim, roll=90.0, hover=0.005, goal_depth=0.038,
                 align_tol=(0.0003, np.radians(0.3)), descend_step=0.004):
        self.X, self.tip, self.rim, self.roll = ee_to_tool, tip, rim, roll
        self.hover, self.goal_depth, self.align_tol, self.descend_step = hover, goal_depth, align_tol, descend_step
        self.phase = "ALIGN"

    def s_for(self, tip_above_rim):
        return self.rim + tip_above_rim - self.tip

    def action(self, obs, cmd_right):
        peg = ee_from_obs(obs, 0) @ self.X["right"]
        hole = ee_from_obs(obs, 1) @ self.X["left"]
        if self.phase == "ALIGN":
            target = hole @ rel_aligned(self.s_for(self.hover), self.roll)
            pe, re = pose_error(peg, target)
            if pe < self.align_tol[0] and re < self.align_tol[1]:
                self.phase = "INSERT"
        if self.phase == "INSERT":
            s_now = (np.linalg.inv(hole) @ peg)[0, 3]
            target = hole @ rel_aligned(max(self.s_for(-self.goal_depth), s_now - self.descend_step), self.roll)
        # steer from the observed EE: cmd moves by (target - observed), so a steady
        # tracking offset is integrated away instead of left in place
        return delta_between(ee_from_obs(obs, 0), target @ np.linalg.inv(self.X["right"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--port-base", type=int, default=7601)
    ap.add_argument("--realtime", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    py = sys.executable
    srv = subprocess.Popen([py, "-m", "ffw_peg_hole_env.server", "--port-base", str(a.port_base)]
                           + (["--realtime"] if a.realtime else []), cwd=ROOT)
    model = MujocoBackend()                       # only for the fixed EE->tool transforms and peg/hole geometry
    X = model.ee_to_tool
    cfg = PegHoleConfig()
    tip, rim = model.tool_geometry()
    ctx = zmq.Context.instance()
    sub_obs = ctx.socket(zmq.SUB)
    sub_obs.setsockopt(zmq.SUBSCRIBE, b"")
    sub_obs.connect(f"tcp://127.0.0.1:{a.port_base + wire.PORT_OBS}")
    pub = ctx.socket(zmq.PUB)
    pub.connect(f"tcp://127.0.0.1:{a.port_base + wire.PORT_CONTROL}")
    sub_st = ctx.socket(zmq.SUB)
    sub_st.setsockopt(zmq.SUBSCRIBE, b"")
    sub_st.connect(f"tcp://127.0.0.1:{a.port_base + wire.PORT_STATUS}")
    sub_img = ctx.socket(zmq.SUB)
    sub_img.setsockopt(zmq.SUBSCRIBE, b"")
    sub_img.connect(f"tcp://127.0.0.1:{a.port_base + wire.PORT_IMAGES}")
    time.sleep(2.0)                               # PUB/SUB join + server start (model + renderer)
    pub.send(b"garbage that is not a frame")     # the server must drop this and keep going
    img_stats = {"ok": 0, "missing": 0, "mismatch": 0, "shape": None}

    def images_for(eid, step, timeout=2.0):
        """Wait for the image message of this (episode, step)."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            if sub_img.poll(100):
                im, _ = wire.decode_images(sub_img.recv_multipart())
                if (im["episode_id"], im["step"]) == (eid, step):
                    img_stats["ok"] += 1
                    img_stats["shape"] = im["right"].shape
                    return im
                if (im["episode_id"], im["step"]) > (eid, step):
                    img_stats["mismatch"] += 1
                    return None
        img_stats["missing"] += 1
        return None

    def wait_status(pred, timeout=10.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if sub_st.poll(100):
                s, _ = wire.decode_status(sub_st.recv())
                if pred(s):
                    return s
        raise TimeoutError("no matching EnvStatus")

    def latest_obs(timeout=5.0):
        if not sub_obs.poll(int(timeout * 1000)):
            raise TimeoutError("no Obs")
        data = sub_obs.recv()
        while sub_obs.poll(0):
            data = sub_obs.recv()
        o, _ = wire.gw.decode_obs(data)
        return o

    results = []
    t0 = time.perf_counter()
    total_steps = 0
    try:
        for ep in range(a.episodes):
            eid = ep + 1
            pub.send(wire.encode_env_cmd(wire.RESET, seed=a.seed + ep, episode_id=eid))
            st = wait_status(lambda s: s["episode_id"] == eid and s["state"] in (wire.READY, wire.FAULT))
            if st["state"] == wire.FAULT:
                print(f"ep {ep}: reset FAULT")
                continue
            obs = latest_obs()
            images_for(eid, 0)
            cmd = ee_from_obs(obs, 0)                       # latch the command reference at READY
            solver = ObsSolver(X, tip, rim)
            step = 0
            while True:
                # clip exactly like the server, so the delta sent is the delta applied
                d = clip_delta(solver.action(obs, cmd), cfg.max_trans, cfg.max_rot)
                cmd = apply_delta(cmd, d)                    # client-side mirror of the server's command
                pub.send(wire.encode_delta((*d, 1.0), (0.0,) * 7))
                st = wait_status(lambda s: s["episode_id"] == eid and s["step"] > step)
                step = st["step"]
                obs = latest_obs()
                images_for(eid, step)
                if st["terminated"] or st["truncated"]:
                    break
            total_steps += step
            results.append(st)
            print(f"ep {ep:3d}: {'OK  ' if st['success'] else 'FAIL'} {step:3d} steps  depth {st['depth'] * 1000:5.1f} mm  "
                  f"lateral {st['lateral'] * 1000:4.2f} mm  contact {st['contact_force']:5.1f} N  "
                  f"deltas received {st['deltas_received']}")
    finally:
        wall = time.perf_counter() - t0
        srv.terminate()
        srv.wait(5)
    ok = sum(r["success"] for r in results)
    print(f"images: {img_stats['ok']} paired with their tick, {img_stats['missing']} missing, "
          f"{img_stats['mismatch']} out of order; shape {img_stats['shape']}")
    print(f"\nover ZMQ ({'realtime 15 Hz' if a.realtime else 'lockstep'}): success {ok}/{len(results)}, "
          f"{total_steps} steps in {wall:.1f} s = {total_steps / wall:.0f} steps/s incl. resets")


if __name__ == "__main__":
    main()
