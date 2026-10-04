#!/usr/bin/env python3
"""Peg moves in a sine wave along the hole axis; the hole (left arm) stays still.

Reset puts the peg aligned on the hole axis, tip --start above the rim. Each
tick the right EE gets a delta of -cap * sin(2 pi k / N) along the hole axis
(peak per-tick delta = the env cap), so the peg goes down and back up once per
N ticks; stroke = 2 * cap * N / (2 pi). Paced at 15 Hz real time by default.
Logs commanded vs actual position along the axis.

  ../ffw_collision_checker/scripts/.venv/bin/python tests/sine_axis.py                 # viewer + wrist cams
  ... --no-view --fast                                                                  # numbers only
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import MujocoBackend, PegHoleConfig, PegHoleTask  # noqa: E402
from ffw_peg_hole_env.peg_hole import rel_aligned  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cap", type=float, default=None, help="peak per-tick delta [m] (default: env cap)")
    ap.add_argument("--period", type=int, default=19, help="ticks per cycle")
    ap.add_argument("--cycles", type=int, default=5)
    ap.add_argument("--start", type=float, default=0.05, help="peg tip above the rim at the top [m]")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episodes", type=int, default=10, help="random resets; each runs --cycles")
    ap.add_argument("--no-view", action="store_true")
    ap.add_argument("--fast", action="store_true", help="don't pace at 15 Hz")
    a = ap.parse_args()

    cfg = PegHoleConfig()
    cap = cfg.max_trans if a.cap is None else a.cap
    view = not a.no_view
    b = MujocoBackend(hz=cfg.hz, timestep=0.004, view=view, cameras=view)
    t = PegHoleTask(b, cfg)
    if view:
        import cv2
    stroke = 2 * cap * a.period / (2 * np.pi)
    print(f"cap {cap * 1000:.1f} mm/tick ({cap * cfg.hz * 100:.0f} cm/s peak), period {a.period} ticks "
          f"({a.period / cfg.hz:.2f} s), stroke {stroke * 100:.1f} cm: tip from {a.start * 100:+.1f} to "
          f"{(a.start - stroke) * 100:+.1f} cm relative to the rim")
    all_err = []
    for ep in range(a.episodes):
        for extra in range(10):                                 # new random hole position
            try:
                t.reset(seed=a.seed + ep + 1000 * extra)
                break
            except RuntimeError as e:
                print(f"ep {ep}: {e} - trying another seed")
        # peg aligned on the axis of this (fixed) hole, tip `start` above the rim
        hole = b.tool_pose("left")
        peg0 = hole @ rel_aligned(t.peg_site_height(a.start), cfg.roll_deg)
        b.teleport({"left": t.T_cmd["left"], "right": t.to_ee("right", peg0)})
        t.T_cmd["right"] = t.to_ee("right", peg0)
        axis = hole[:3, 0]                                      # hole axis (tool x) in base_link, up
        s0 = t.insertion_state()["height"]
        rows = []
        period = 1.0 / cfg.hz
        t_next = time.monotonic()
        for k in range(a.period * a.cycles):
            d = -cap * np.sin(2 * np.pi * k / a.period)
            obs, r, term, trunc, info = t.step(np.concatenate([d * axis, np.zeros(3)]), np.zeros(6))
            cmd_s = float((np.linalg.inv(hole) @ t.T_cmd["right"] @ b.ee_to_tool["right"])[0, 3]) - s0
            ins = t.insertion_state()
            rows.append((k, d, cmd_s, ins["height"] - s0, ins["lateral"], obs["contact_force"]))
            if view:
                im = b.render()
                pair = cv2.resize(np.hstack([im["right"], im["left"]]), None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
                cv2.imshow("wrist cameras: right (peg) | left (hole)", cv2.cvtColor(pair, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
            if not a.fast:
                t_next += period
                time.sleep(max(0.0, t_next - time.monotonic()))
        R = np.array(rows)
        err = (R[:, 3] - R[:, 2]) * 1000
        all_err.append(err)
        print(f"ep {ep}: hole axis {np.round(axis, 3)}, tracking max |e| {np.abs(err).max():.2f} mm, "
              f"lateral max {R[:, 4].max() * 1000:.2f} mm, contact max {R[:, 5].max():.1f} N")
    err = np.concatenate(all_err)
    print(f"along the axis: command range {R[:, 2].min() * 100:+.2f} .. {R[:, 2].max() * 100:+.2f} cm, "
          f"actual {R[:, 3].min() * 100:+.2f} .. {R[:, 3].max() * 100:+.2f} cm")
    print(f"tracking error actual - command: max |e| {np.abs(err).max():.2f} mm, rms {np.sqrt(np.mean(err ** 2)):.2f} mm; "
          f"lateral max {R[:, 4].max() * 1000:.2f} mm; peg-hole contact max {R[:, 5].max():.1f} N")
    for k in range(0, len(R), max(1, a.period // 4)):
        print(f"  tick {int(R[k, 0]):3d}: delta {R[k, 1] * 1000:+6.2f} mm  cmd {R[k, 2] * 1000:+7.2f}  "
              f"actual {R[k, 3] * 1000:+7.2f}  err {err[k]:+5.2f} mm")
    if view:
        print("close the viewer window to exit")
    b.close()


if __name__ == "__main__":
    main()
