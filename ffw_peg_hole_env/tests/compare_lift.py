#!/usr/bin/env python3
"""Same grid as sweep_box_view.py, headless: scripted insertion at every grid
point of the reset box, with the lift hard-locked vs free in the IK (both arms
+ lift solved together, lift cost --lift-weight). Per point: peg-hole contact
force (peak, mean over the insertion), steps, peak off-axis and tilt, how far
the lift and the hole moved.

  ../ffw_collision_checker/scripts/.venv/bin/python tests/compare_lift.py
  --weights 10 1   lift cost(s) for the free runs
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ffw_peg_hole_env import InsertionSolver, MujocoBackend, PegHoleConfig, PegHoleTask  # noqa: E402
from ffw_peg_hole_env.sim_backend import DEFAULT_LIFT  # noqa: E402
from ffw_peg_hole_env.geometry import T_from, delta_between  # noqa: E402
from ffw_peg_hole_env.peg_hole import rel_aligned  # noqa: E402
from sweep_box_view import reset_at  # noqa: E402


class AimedSolver(InsertionSolver):
    """InsertionSolver aiming at the hole shifted `err` across its axis (hole site y/z):
    a calibration / perception error, so the peg actually meets the walls."""

    def __init__(self, task, err):
        super().__init__(task)
        self.err = err

    def action(self):
        t = self.task
        hole = t.b.tool_pose("left") @ T_from([0.0, *self.err], np.eye(3))
        roll = t.cfg.roll_deg
        if self.phase == "ALIGN":
            target = hole @ rel_aligned(t.peg_site_height(self.hover), roll)
            pe = np.linalg.norm(t.b.tool_pose("right")[:3, 3] - target[:3, 3])
            if pe < self.align_tol[0] * 3:
                self.phase = "INSERT"
        if self.phase == "INSERT":
            s_now = t.insertion_state()["height"]
            target = hole @ rel_aligned(max(t.peg_site_height(-self.goal_depth), s_now - self.descend_step), roll)
        return delta_between(t.b.site_pose("right"), t.to_ee("right", target))


def run(lift, lift_free, weight, grid, tip, timestep, aim_err=0.0):
    b = MujocoBackend(cameras=False, timestep=timestep, lift=lift, lift_free=lift_free, lift_weight=weight)
    task = PegHoleTask(b, PegHoleConfig())
    lq = lambda: b.d.qpos[b.lift_qadr]
    rows = []
    rng = np.random.default_rng(0)                      # same error directions in every mode
    for dh in grid:
        ang = rng.uniform(0, 2 * np.pi)
        solver = AimedSolver(task, aim_err * np.array([np.cos(ang), np.sin(ang)]))
        reset_at(task, dh, tip)
        solver.reset()
        hole0 = b.tool_pose("left")[:3, 3].copy()
        F, lat, tilt, dl, dhole = [], [], [], [], []
        while True:
            obs, r, term, trunc, info = task.step(solver.action())
            ins = task.insertion_state()
            F.append(b.peg_hole_force())
            lat.append(ins["lateral"])
            tilt.append(ins["tilt_deg"])
            dl.append(abs(lq() - lift))
            dhole.append(np.linalg.norm(b.tool_pose("left")[:3, 3] - hole0))
            if term or trunc:
                break
        rows.append((info["success"], info["steps"], max(F), float(np.mean(F)), max(lat), max(tilt),
                     max(dl), max(dhole)))
    return np.array(rows, dtype=float)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--nz", type=int, default=3)
    ap.add_argument("--lift", type=float, default=DEFAULT_LIFT)
    ap.add_argument("--weights", type=float, nargs="+", default=[10.0, 1.0])
    ap.add_argument("--tip", type=float, default=0.03)
    ap.add_argument("--timestep", type=float, default=0.004)
    ap.add_argument("--aim-errors", type=float, nargs="+", default=[0.0, 0.0005, 0.001, 0.0015],
                    help="m, the insertion aims this far off the true hole axis (random direction per point)")
    a = ap.parse_args()
    c = PegHoleConfig()
    xs = np.linspace(-c.hole_xy, c.hole_xy, a.n)
    zs = np.linspace(c.hole_z[1], c.hole_z[0], a.nz)
    grid = [np.array([x, y, z]) for z in zs for y in xs for x in xs]
    print(f"{len(grid)} grid points, lift {a.lift:+.2f} m\n")
    hdr = (f"{'mode':22s} {'ok':>6s} {'steps':>6s} {'peak F N':>15s} {'mean F N':>9s} {'off-axis mm':>12s} "
           f"{'tilt deg':>9s} {'lift moved mm':>14s} {'hole moved mm':>14s}")
    print(hdr)
    modes = [("lift locked", False, 1.0)] + [(f"lift free (cost {w:g})", True, w) for w in a.weights]
    for err in a.aim_errors:
        print(f"-- aim error {err * 1000:.1f} mm")
        for name, free, w in modes:
            R = run(a.lift, free, w, grid, a.tip, a.timestep, err)
            print(f"{name:22s} {int(R[:, 0].sum()):3d}/{len(R):<2d} {np.median(R[:, 1]):6.0f} "
                  f"{np.median(R[:, 2]):6.1f} / {R[:, 2].max():6.1f} {np.median(R[:, 3]):9.2f} "
                  f"{np.median(R[:, 4]) * 1000:5.2f} / {R[:, 4].max() * 1000:4.2f} {R[:, 5].max():9.2f} "
                  f"{np.median(R[:, 6]) * 1000:6.2f} / {R[:, 6].max() * 1000:5.2f} "
                  f"{np.median(R[:, 7]) * 1000:6.2f} / {R[:, 7].max() * 1000:5.2f}")
    print("\n(peak F, off-axis, lift moved, hole moved: median / max over the grid)")


if __name__ == "__main__":
    main()
