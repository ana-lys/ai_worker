#!/usr/bin/env python3
"""Stepwise vs continuous insertion (peg_hole_teach.py's two motion modes) in
the MuJoCo sim, made robot-like: a 100 Hz control loop, the robot's ~0.28 s
command-to-motion lag (the sim's own ~0.03 s + a --delay FIFO), and the teach
tool's integral position correction (gain 0.5/s, 2/s across the axis during
the push).

  stepwise   : to CLEAR above the axis (5 cm/s), settle; down to ON_TOP (1 cm/s),
               settle; wait until the height is within 0.5 mm (<= 3 s); 0.5 s
               still; push (1 cm/s); hold at the bottom (<= 1.5 s)
  continuous : one timed path start -> above the axis -> ON_TOP -> bottom, no
               stops, command --lead s ahead, correction across the motion only

Per episode: success, time from the start to the bottom, how far below ON_TOP
the peg overshoots before the push (stepwise) / its height error passing ON_TOP
(continuous), peak and mean peg-hole contact force. --aim-errors aims the
insertion off the true hole axis (random direction) to force contact.

  ../ffw_collision_checker/scripts/.venv/bin/python tests/continuous_vs_stepwise.py
"""
import argparse
import collections
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import MujocoBackend, PegHoleConfig, PegHoleTask  # noqa: E402
from ffw_peg_hole_env.geometry import T_from  # noqa: E402
from ffw_peg_hole_env.peg_hole import rel_aligned  # noqa: E402

DT = 0.01


class LaggyRobot:
    """The sim arm driven like the real one: peg-site goals at 100 Hz -> + integral
    correction -> a delay FIFO -> IK -> physics. The left arm holds the hole."""

    def __init__(self, task, delay, gui=None):
        self.t, self.b = task, task.b
        self.gui = gui                               # GUI: paces ticks and draws the overlay
        self.fifo = collections.deque()
        self.n_delay = int(round(delay / DT))
        self.corr = np.zeros(3)
        self.goal = None
        self.F = []

    def real(self):
        return self.b.tool_pose("right")

    def height(self, T):
        return (np.linalg.inv(self.t.hole_site) @ T)[0, 3]

    def tick(self, goal, gain, free_dir=None):
        self.goal = goal
        e = goal[:3, 3] - self.real()[:3, 3]
        if free_dir is not None:
            e = e - (e @ free_dir) * free_dir
        self.corr = self.corr + gain * DT * e
        n = np.linalg.norm(self.corr)
        if n > 0.03:
            self.corr *= 0.03 / n
        cmd = goal.copy()
        cmd[:3, 3] += self.corr
        self.fifo.append(cmd)
        if len(self.fifo) > self.n_delay:
            cmd = self.fifo.popleft()
        else:
            cmd = self.fifo[0]
        self.t.T_cmd["right"] = self.t.to_ee("right", cmd)
        t0 = time.perf_counter()
        self.b.command_all(self.t.T_cmd)
        self.b.tick()
        self.F.append(self.b.peg_hole_force())
        if self.gui is not None:
            self.gui.frame(self)
            time.sleep(max(0.0, DT / self.gui.speed - (time.perf_counter() - t0)))


class Gui:
    """MuJoCo viewer overlay: mode / time / height error / contact force text, the goal
    (blue dot), the delayed command actually executed (orange) and the peg (green)."""

    def __init__(self, b, speed):
        import mujoco
        self.mj, self.b, self.speed = mujoco, b, speed
        self.label, self.t0, self.path = "", 0.0, []

    def start(self, label, path=()):
        self.label, self.t0, self.path = label, time.perf_counter(), list(path)

    def frame(self, rob):
        mj, v = self.mj, self.b.viewer
        if v is None or not v.is_running():
            raise SystemExit("viewer closed")
        real = rob.real()[:3, 3]
        dots = [(p, (0.3, 0.5, 1.0, 0.35), 0.0012) for p in self.path[::8]]
        dots += [(rob.goal[:3, 3], (0.2, 0.4, 1.0, 1.0), 0.003),
                 (self.b.tool_pose("right")[:3, 3], (0.1, 0.9, 0.2, 1.0), 0.0025)]
        with v.lock():
            scn = v.user_scn
            scn.ngeom = 0
            for p, rgba, r in dots:
                if scn.ngeom >= scn.maxgeom:
                    break
                mj.mjv_initGeom(scn.geoms[scn.ngeom], mj.mjtGeom.mjGEOM_SPHERE, np.array([r, 0, 0]), p,
                                np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
                scn.ngeom += 1
        dh = (rob.height(rob.goal) - rob.height(rob.real())) * 1000
        v.set_texts((mj.mjtFontScale.mjFONTSCALE_150, mj.mjtGridPos.mjGRID_TOPLEFT,
                     f"{self.label}\ntime\ncontact force\ngoal - peg along axis",
                     f"\n{time.perf_counter() - self.t0:5.1f} s\n{rob.F[-1]:5.1f} N\n{dh:+5.1f} mm"))


def segment(A, B, speed, rot_speed=math.radians(20)):
    lin = np.linalg.norm(B[:3, 3] - A[:3, 3])
    ang = np.linalg.norm((Rot.from_matrix(B[:3, :3]) * Rot.from_matrix(A[:3, :3]).inv()).as_rotvec())
    n = max(1, int(math.ceil(max(lin / speed, ang / rot_speed) / DT)))
    sl = Slerp([0, 1], Rot.from_matrix([A[:3, :3], B[:3, :3]]))
    return [T_from((1 - u) * A[:3, 3] + u * B[:3, 3], sl(u).as_matrix()) for u in (np.arange(1, n + 1) / n)]


def continuous_path(G0, W1, W2, s_of, top, a):
    """Same profile as peg_hole_teach.Teach.continuous_path (peg-site poses)."""
    step = 0.0002
    P, R_, V = [], [], []
    lin = float(np.linalg.norm(W1[:3, 3] - G0[:3, 3]))
    ang = float(np.linalg.norm((Rot.from_matrix(W1[:3, :3]) * Rot.from_matrix(G0[:3, :3]).inv()).as_rotvec()))
    L1 = max(lin, ang / math.radians(20) * a.move_speed, 1e-6)
    sl = Slerp([0, 1], Rot.from_matrix([G0[:3, :3], W1[:3, :3]]))
    n1 = max(1, int(math.ceil(L1 / step)))
    for i in range(n1):
        u = i / n1
        P.append((1 - u) * G0[:3, 3] + u * W1[:3, 3]); R_.append(sl(u).as_matrix()); V.append(a.move_speed * lin / L1)
    L2 = float(np.linalg.norm(W2[:3, 3] - W1[:3, 3]))
    n2 = max(1, int(math.ceil(L2 / step)))
    for i in range(n2 + 1):
        u = i / n2
        pt = (1 - u) * W1[:3, 3] + u * W2[:3, 3]
        P.append(pt); R_.append(W1[:3, :3])
        V.append(a.approach_speed if s_of(pt) > top + 0.003 else a.speed)
    V[n1] = min(V[n1], a.speed)
    P = np.array(P)
    dp = np.r_[0.0, np.full(n1, L1 / n1), np.full(n2, L2 / n2)][:len(P)]
    v = np.array(V, float)
    v[0] = v[-1] = 0.0
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a.accel * dp[i]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a.accel * dp[i + 1]))
    t = np.cumsum(np.r_[0.0, 2 * dp[1:] / np.maximum(v[1:] + v[:-1], 1e-6)])
    dirs = np.gradient(P, axis=0)
    dirs /= np.maximum(np.linalg.norm(dirs, axis=1, keepdims=True), 1e-12)
    return t, [T_from(p, r) for p, r in zip(P, R_)], dirs


def episode(task, seed, mode, aim, a, gui=None):
    task.reset(seed=seed)
    rob = LaggyRobot(task, a.delay, gui)
    H_true = task.hole_site
    ang = np.random.default_rng(seed + 999).uniform(0, 2 * np.pi)
    H = H_true @ T_from([0.0, aim * np.cos(ang), aim * np.sin(ang)], np.eye(3))   # where we aim
    roll = task.cfg.roll_deg
    s_top = task.peg_site_height(0.0005)
    aligned = lambda s: H @ rel_aligned(s, roll)
    s_of_pt = lambda p: (np.linalg.inv(H) @ np.r_[p, 1.0])[0]
    G0 = rob.real()
    bottom = aligned(s_top - a.push)
    if gui is not None:
        gui.start(f"{mode.upper()}  (reset {seed}, aim error {aim * 1000:.1f} mm)")
    axis = H[:3, 0]
    ticks = 0
    over = np.nan
    if mode == "stepwise":
        for T in segment(G0, aligned(s_top + 0.02), a.move_speed):
            rob.tick(T, 0.5); ticks += 1
        for _ in range(30):
            rob.tick(rob.goal, 0.5); ticks += 1
        for T in segment(rob.goal, aligned(s_top), a.speed):
            rob.tick(T, 0.5); ticks += 1
        for _ in range(30):
            rob.tick(rob.goal, 0.5); ticks += 1
        hs = []
        for _ in range(300):                                   # settle_height <= 3 s
            rob.tick(rob.goal, 0.5); ticks += 1
            hs.append(rob.height(rob.real()) - s_top)
            if abs(hs[-1]) <= 0.0005:
                break
        over = -min(hs) * 1000
        for _ in range(50):
            rob.tick(rob.goal, 0.5); ticks += 1
        for T in segment(rob.goal, bottom, a.speed):
            rob.tick(T, 2.0, axis); ticks += 1
    else:
        t, poses, dirs = continuous_path(G0, aligned(min(s_top + 0.02, max(rob.height(G0), s_top + 0.002))), bottom,
                                         s_of_pt, s_top, a)
        if gui is not None:
            gui.path = [P[:3, 3] for P in poses]
        errs = []
        k = 0
        while k * DT <= t[-1]:
            i_cmd = min(int(np.searchsorted(t, k * DT + a.lead)), len(poses) - 1)
            i_ref = min(int(np.searchsorted(t, k * DT)), len(poses) - 1)
            rob.tick(poses[i_cmd], 2.0, dirs[i_ref]); ticks += 1
            if abs(s_of_pt(poses[i_ref][:3, 3]) - s_top) < 0.0005:
                errs.append((rob.height(rob.real()) - s_of_pt(poses[i_ref][:3, 3])) * 1000)
            k += 1
        over = float(np.max(np.abs(errs))) if errs else np.nan
    for _ in range(150):                                       # bottom hold <= 1.5 s
        if rob.height(rob.real()) - rob.height(bottom) <= 0.0005:
            break
        rob.tick(bottom, 2.0, axis); ticks += 1
    depth = task.insertion_state()["depth"]
    return {"ok": depth >= task.cfg.success_depth, "time": ticks * DT, "over_mm": over,
            "Fpeak": max(rob.F), "Fmean": float(np.mean(rob.F)), "depth_mm": depth * 1000}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--delay", type=float, default=0.25, help="s added to the sim's own ~0.03 s lag")
    ap.add_argument("--lead", type=float, default=0.28)
    ap.add_argument("--aim-errors", type=float, nargs="+", default=[0.0, 0.0005, 0.001])
    ap.add_argument("--push", type=float, default=0.022, help="m below ON_TOP (sim hole depth)")
    ap.add_argument("--move-speed", type=float, default=0.05)
    ap.add_argument("--approach-speed", type=float, default=0.02)
    ap.add_argument("--speed", type=float, default=0.01)
    ap.add_argument("--accel", type=float, default=0.05)
    ap.add_argument("--view", action="store_true", help="GUI: each reset runs stepwise then continuous in the viewer")
    ap.add_argument("--rate", type=float, default=1.0, help="GUI: x real time")
    ap.add_argument("--pause", type=float, default=1.0, help="GUI: s between episodes")
    a = ap.parse_args()
    if a.view:
        os.environ["MUJOCO_GL"] = "glfw"
    b = MujocoBackend(hz=100.0, cameras=False, timestep=0.002, view=a.view)
    if a.view:
        gui = Gui(b, a.rate)
        task = PegHoleTask(b, PegHoleConfig())
        try:
            for seed in range(a.episodes):
                for aim in a.aim_errors:
                    for mode in ("stepwise", "continuous"):
                        r = episode(task, seed, mode, aim, a, gui)
                        print(f"reset {seed} aim {aim * 1000:.1f} mm {mode:10s}: {'OK  ' if r['ok'] else 'FAIL'} "
                              f"{r['time']:5.2f} s, top err {r['over_mm']:.2f} mm, peak {r['Fpeak']:.1f} N")
                        time.sleep(a.pause)
        except SystemExit as e:
            print(e)
        b.close()
        return
    task = PegHoleTask(b, PegHoleConfig())
    print(f"{a.episodes} random resets, lag {a.delay + 0.03:.2f} s, push {a.push * 1000:.0f} mm\n")
    print(f"{'aim err':>7s} {'mode':10s} {'ok':>6s} {'time s':>7s} {'top err mm':>11s} {'peak F N':>15s} {'mean F N':>9s}")
    for aim in a.aim_errors:
        for mode in ("stepwise", "continuous"):
            R = [episode(task, s, mode, aim, a) for s in range(a.episodes)]
            ok = sum(r["ok"] for r in R)
            col = lambda k: np.array([r[k] for r in R])
            print(f"{aim * 1000:5.1f}mm {mode:10s} {ok:3d}/{len(R):<2d} {np.median(col('time')):7.2f} "
                  f"{np.nanmedian(col('over_mm')):5.2f}/{np.nanmax(col('over_mm')):4.2f} "
                  f"{np.median(col('Fpeak')):6.1f} / {col('Fpeak').max():6.1f} {np.median(col('Fmean')):9.2f}")
    print("\ntop err: stepwise = overshoot below ON_TOP before the push; continuous = height error passing ON_TOP "
          "(median / max)")


if __name__ == "__main__":
    main()
