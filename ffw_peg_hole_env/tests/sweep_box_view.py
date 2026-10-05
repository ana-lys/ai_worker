#!/usr/bin/env python3
"""Watch the reset region: a transparent box = where RESET samples the hole
(hole frame, z = insertion axis: x/y +-hole_xy, z hole_z), and a scripted
insertion at every point of a grid inside it, at --speed x real time.

Grid points are drawn as dots: grey = to do, yellow = current, green = inserted,
red = failed. The arms travel between points (peg backs out, the left arm carries the hole to
the next point with the peg hovering aligned --tip above the rim, insert);
--teleport jumps instead.

  MUJOCO_GL=glfw ../ffw_collision_checker/scripts/.venv/bin/python tests/sweep_box_view.py
  --n 5 --nz 3 --speed 10 --lift -0.3
"""
import argparse
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "glfw")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import InsertionSolver, MujocoBackend, PegHoleConfig, PegHoleTask  # noqa: E402
from ffw_peg_hole_env.geometry import SITE_TO_ZUP, T_from, pos, rot, zup  # noqa: E402
from ffw_peg_hole_env.peg_hole import rel_aligned  # noqa: E402
from ffw_peg_hole_env.sim_backend import DEFAULT_LIFT  # noqa: E402

GREY, YELLOW, GREEN, RED = (0.6, 0.6, 0.6, 0.6), (1.0, 0.85, 0.0, 1.0), (0.1, 0.9, 0.2, 0.9), (1.0, 0.1, 0.1, 0.9)


def reset_at(task, dh, tip):
    """RESET with the hole at offset dh (hole frame) and the peg aligned tip above the rim."""
    H_nom = zup(task.hole_nominal)
    H = H_nom.copy()
    H[:3, 3] = pos(H_nom) + rot(H_nom) @ dh
    hole_site = T_from(pos(H), rot(H) @ SITE_TO_ZUP.T)
    peg = hole_site @ rel_aligned(task.peg_site_height(tip), task.cfg.roll_deg)
    err = task.b.teleport({"left": task.to_ee("left", hole_site), "right": task.to_ee("right", peg)})
    task.hole_site = hole_site
    task.T_cmd = {"left": task.to_ee("left", hole_site), "right": task.to_ee("right", peg)}
    task.steps = 0
    return err


def hole_at(task, dh):
    H_nom = zup(task.hole_nominal)
    return T_from(pos(H_nom) + rot(H_nom) @ dh, rot(H_nom) @ SITE_TO_ZUP.T)


def travel(task, hole_to, tip, speed, dt, running):
    """Move instead of teleporting: the peg backs out straight up the axis to `tip`
    above the rim, then the left arm carries the hole to hole_to with the peg
    hovering aligned above it. 15 Hz ticks, `speed` m/s of sim time."""
    b, roll = task.b, task.cfg.roll_deg
    above = lambda H: H @ rel_aligned(task.peg_site_height(tip), roll)

    def go(path):
        for H, P in path:
            if not running():
                return
            t0 = time.perf_counter()
            task.T_cmd = {"left": task.to_ee("left", H), "right": task.to_ee("right", P)}
            for side in ("left", "right"):
                b.command(side, task.T_cmd[side])
            b.tick()
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))

    def line(A, B):
        n = max(1, int(np.ceil(np.linalg.norm(pos(B) - pos(A)) / (speed / 15.0))))
        return [T_from(pos(A) + (pos(B) - pos(A)) * k / n, rot(B)) for k in range(1, n + 1)]

    H0 = task.hole_site
    P0 = task.T_cmd["right"] @ task.b.ee_to_tool["right"]          # commanded peg site
    go([(H0, P) for P in line(P0, above(H0))])                     # back out along the axis
    go([(H, above(H)) for H in line(H0, hole_to)])                 # carry the hole, peg hovering
    task.hole_site = hole_to
    task.steps = 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=5, help="grid points per across axis")
    ap.add_argument("--nz", type=int, default=3, help="grid points along the insertion axis")
    ap.add_argument("--speed", type=float, default=10.0, help="x real time (15 Hz ticks)")
    ap.add_argument("--lift", type=float, default=DEFAULT_LIFT)
    ap.add_argument("--tip", type=float, default=0.03, help="m, peg tip above the rim at the start")
    ap.add_argument("--timestep", type=float, default=0.004)
    ap.add_argument("--travel-speed", type=float, default=0.05, help="m/s (sim time) between grid points")
    ap.add_argument("--teleport", action="store_true", help="jump between grid points instead of moving")
    a = ap.parse_args()

    b = MujocoBackend(hz=15.0, view=True, timestep=a.timestep, cameras=False, lift=a.lift)
    with b.viewer.lock():
        b.viewer.opt.sitegroup[:] = 0                                    # hide every site (all 6 groups)
    task = PegHoleTask(b, PegHoleConfig())
    solver = InsertionSolver(task)
    c = task.cfg
    H_nom = zup(task.hole_nominal)

    xs = np.linspace(-c.hole_xy, c.hole_xy, a.n)
    zs = np.linspace(c.hole_z[1], c.hole_z[0], a.nz)
    grid = [np.array([x, y, z]) for z in zs for y in xs for x in xs]
    status = [GREY] * len(grid)

    # the box: centre and half sizes in the hole frame
    box_c = np.array([0.0, 0.0, 0.5 * (c.hole_z[0] + c.hole_z[1])])
    box_h = np.array([c.hole_xy, c.hole_xy, 0.5 * (c.hole_z[1] - c.hole_z[0])])
    R = rot(H_nom)

    def draw(cur=None):
        scn = b.viewer.user_scn
        with b.viewer.lock():
            scn.ngeom = 0
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, box_h, pos(H_nom) + R @ box_c, R.flatten(),
                                np.array([0.2, 0.8, 0.3, 0.15], dtype=np.float32))
            scn.ngeom += 1
            for i, p in enumerate(grid):
                if scn.ngeom >= scn.maxgeom:
                    break
                rgba = YELLOW if i == cur else status[i]
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                    np.array([0.003 if i != cur else 0.005, 0, 0]),
                                    pos(H_nom) + R @ p, np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
                scn.ngeom += 1

    dt = 1.0 / (15.0 * a.speed)
    ok = 0
    print(f"lift {a.lift:+.2f} m, {len(grid)} grid points in the box (+-{c.hole_xy * 100:.0f} cm across, "
          f"{c.hole_z[0] * 100:.0f}..{c.hole_z[1] * 100:.0f} cm along the axis), {a.speed:g}x real time")
    for i, dh in enumerate(grid):
        if not b.viewer.is_running():
            break
        draw(i)
        if i == 0 or a.teleport:
            pe, re = reset_at(task, dh, a.tip)
        else:
            travel(task, hole_at(task, dh), a.tip, a.travel_speed, dt, b.viewer.is_running)
            pe = 0.0
        solver.reset()
        b.viewer.sync()
        res = None
        while b.viewer.is_running():
            t0 = time.perf_counter()
            obs, r, term, trunc, info = task.step(solver.action())
            if term or trunc:
                res = info
                break
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
        if res is None:
            break
        status[i] = GREEN if res["success"] else RED
        ok += res["success"]
        print(f"  {i + 1:3d}/{len(grid)} hole offset ({dh[0] * 100:+.1f}, {dh[1] * 100:+.1f}, {dh[2] * 100:+.1f}) cm: "
              f"{'OK  ' if res['success'] else 'FAIL'} in {res['steps']} steps")
    draw()
    b.viewer.sync()
    print(f"\n{ok}/{i + 1} inserted. Close the viewer window to exit.")
    b.close()


if __name__ == "__main__":
    main()
