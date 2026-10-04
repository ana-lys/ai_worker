#!/usr/bin/env python3
"""Run N sim episodes: reset(seed) -> scripted solver -> step until done; report
success, steps, contact force and speed.

  ../ffw_collision_checker/scripts/.venv/bin/python tests/run_sim_insertion.py --episodes 50
  ... --view          # watch in the MuJoCo viewer
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import InsertionSolver, MujocoBackend, PegHoleConfig, PegHoleTask  # noqa: E402
from ffw_peg_hole_env.sim_backend import DEFAULT_LIFT  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--timestep", type=float, default=None, help="physics timestep (implicitfast); default = model's 2 ms")
    ap.add_argument("--cameras", action="store_true", help="render both wrist cameras every step")
    ap.add_argument("--lift", type=float, default=DEFAULT_LIFT, help="lift_joint [m], held fixed (0 = top)")
    ap.add_argument("--show-cams", action="store_true", help="show the wrist images in a window (implies --cameras)")
    ap.add_argument("--realtime", action="store_true", help="pace steps at 15 Hz (for watching)")
    ap.add_argument("--pause", type=float, default=0.0, help="s to pause after each episode (for watching)")
    a = ap.parse_args()
    a.cameras = a.cameras or a.show_cams
    if a.show_cams:
        import cv2

    backend = MujocoBackend(hz=15.0, view=a.view, timestep=a.timestep, cameras=a.cameras, lift=a.lift)
    task = PegHoleTask(backend, PegHoleConfig())
    solver = InsertionSolver(task)
    print(f"peg tip {task.tip * 1000:+.1f} mm along the peg site x, hole rim {task.rim * 1000:+.1f} mm "
          f"along the hole site x; {backend.nsub} physics steps per {1000 / backend.hz:.1f} ms tick")

    results = []
    t_steps = 0.0
    n_steps = 0
    t0 = time.perf_counter()
    for ep in range(a.episodes):
        obs, info = task.reset(seed=a.seed + ep)
        solver.reset()
        fmax = 0.0
        while True:
            ts = time.perf_counter()
            obs, r, term, trunc, sinfo = task.step(solver.action())
            if a.cameras:
                imgs = backend.render()
                if a.show_cams:
                    pair = np.hstack([imgs["right"], imgs["left"]])
                    pair = cv2.resize(pair, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
                    cv2.imshow("wrist cameras: right (peg) | left (hole)", cv2.cvtColor(pair, cv2.COLOR_RGB2BGR))
                    cv2.waitKey(1)
            t_steps += time.perf_counter() - ts
            n_steps += 1
            fmax = max(fmax, obs["contact_force"])
            if a.realtime:
                time.sleep(max(0.0, 1.0 / backend.hz - (time.perf_counter() - ts)))
            if term or trunc:
                break
        ins = obs["insertion"]
        if a.pause > 0:
            time.sleep(a.pause)
        results.append((sinfo["success"], sinfo["steps"], fmax, ins["lateral"], ins["depth"]))
        print(f"ep {ep:3d} seed {a.seed + ep:3d}: {'OK  ' if sinfo['success'] else ('FAIL' if sinfo['failed'] else 'TIME')} "
              f"{sinfo['steps']:3d} steps ({sinfo['steps'] / 15:.1f} s sim)  depth {ins['depth'] * 1000:5.1f} mm  "
              f"lateral {ins['lateral'] * 1000:4.2f} mm  max contact {fmax:6.1f} N  "
              f"(hole {np.round(info['hole_offset'] * 100, 1)} cm, peg {np.round(info['peg_offset'] * 100, 1)} cm, "
              f"tilt {np.round(info['peg_tilt_deg'], 1)} deg, reset tries {info['reset_attempts']})")
    wall = time.perf_counter() - t0
    ok = [r for r in results if r[0]]
    print(f"\nsuccess {len(ok)}/{len(results)}; steps median {np.median([r[1] for r in ok]) if ok else float('nan'):.0f}; "
          f"max contact median {np.median([r[2] for r in results]):.1f} N")
    print(f"speed: {n_steps} steps in {t_steps:.2f} s of step() = {n_steps / t_steps:.0f} steps/s "
          f"({n_steps / t_steps / 15:.0f}x real time at 15 Hz); wall incl. resets {wall:.1f} s")
    if a.view:
        print("close the viewer window to exit")
    backend.close()


if __name__ == "__main__":
    main()
