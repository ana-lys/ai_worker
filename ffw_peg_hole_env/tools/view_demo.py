#!/usr/bin/env python3
"""View recorded peg-hole demos: both wrist images, the frame's numbers and a force / depth plot.

  python view_demo.py demos                     # a demo dir (episodes/*.npz) or a single .npz
  python view_demo.py demos --speed 2 --start 17

Keys (window "peg-hole demo"):
  space  play / pause            + / -   speed x2 / x0.5  (0.125 .. 16x; 1x = real time)
  . / ,  next / previous frame   n / p   next / previous episode, at the same time into it (closest frame)
  r      restart episode         q / Esc quit
The slider scrubs the episode.
"""
import argparse
import glob
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve()
for p in (HERE.parents[1], HERE.parents[1] / "ffw_peg_hole_env"):     # kit root (wire.py) or the repo package
    sys.path.insert(0, str(p))
import wire  # noqa: E402

WIN = "peg-hole demo"
SCALE = 3                                        # 128 px images shown at 384
PLOT_H = 170


def load(path):
    z = np.load(path)
    F = [wire.decode_frame(r.tobytes()) for r in z["frames"]]
    meta = json.loads(str(z["meta"]))
    series = {k: np.array([np.nan_to_num(f[1][k], nan=0.0) for f in F]) for k in ("push_back", "depth", "lateral")}
    return {"frames": F, "right": z["img_right"], "left": z["img_left"], "meta": meta, "series": series,
            "ts": np.array([f[3] for f in F]), "name": os.path.basename(path)}


def plot(ep, k, w):
    """Push-back (N, orange) and depth (mm, blue; 0 = rim) over the episode, cursor at frame k."""
    img = np.full((PLOT_H, w, 3), 25, np.uint8)
    n = len(ep["frames"])
    x = (np.arange(n) * (w - 20) / max(n - 1, 1) + 10).astype(int)
    def line(v, lo, hi, col):
        y = (PLOT_H - 15 - (np.clip(v, lo, hi) - lo) / (hi - lo) * (PLOT_H - 30)).astype(int)
        cv2.polylines(img, [np.stack([x, y], 1).reshape(-1, 1, 2)], False, col, 1, cv2.LINE_AA)
        return y
    line(np.zeros(n), -40, 40, (70, 70, 70))                              # zero line (shared scale)
    line(ep["series"]["depth"] * 1000, -40, 40, (230, 160, 60))
    line(ep["series"]["push_back"] * 4, -40, 40, (60, 150, 255))         # x4 so 10 N spans like 40 mm
    for i, f in enumerate(ep["frames"]):                                   # intervention ticks
        if f[2]["frame_state"] == wire.FS_INTERVENTION:
            cv2.line(img, (x[i], PLOT_H - 12), (x[i], PLOT_H - 6), (0, 0, 255), 1)
    cv2.line(img, (x[k], 5), (x[k], PLOT_H - 5), (255, 255, 255), 1)
    cv2.putText(img, "depth mm (blue, -40..40)   push-back N x4 (orange)   red ticks = intervention",
                (10, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
    return img


def render(ep, k, speed, playing, idx, n_ep):
    obs, priv, tag, ts = ep["frames"][k]
    tiles = [cv2.resize(ep[c][k][:, :, ::-1], None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)
             for c in ("right", "left")]
    for t, lab in zip(tiles, ("right wrist (peg)", "left wrist (hole)")):
        cv2.putText(t, lab, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    top = np.hstack(tiles)
    w = top.shape[1]
    st = wire.FS_NAMES[tag["frame_state"]] + (f" {wire.TR_NAMES[tag['reason']]}" if tag["frame_state"] == wire.FS_TERMINATED else "")
    a = tag["action"]
    lines = [f"{ep['name']}  ({idx + 1}/{n_ep})   step {tag['step']}/{len(ep['frames']) - 1}   t {ts - ep['ts'][0]:5.2f} s   "
             f"{'PLAY' if playing else 'PAUSE'} x{speed:g}",
             f"{st:14s} reward {tag['reward']:+.3f}  return {priv['return']:+.3f}  interventions {priv['interventions']:.0f}",
             f"depth {priv['depth'] * 1000:+6.1f} mm  lateral {priv['lateral'] * 1000:5.2f} mm  tilt {priv['tilt']:4.2f} deg",
             f"push-back {priv['push_back']:+5.1f} N  (peak {priv['push_back_peak']:4.1f} N)",
             f"action  d {a[0] * 1000:+5.2f} {a[1] * 1000:+5.2f} {a[2] * 1000:+5.2f} mm   r {np.degrees(a[3]):+5.2f} "
             f"{np.degrees(a[4]):+5.2f} {np.degrees(a[5]):+5.2f} deg",
             "space play/pause  +/- speed  ,/. step  n/p episode  r restart  q quit"]
    txt = np.full((26 * len(lines) + 10, w, 3), 15, np.uint8)
    for i, s in enumerate(lines):
        cv2.putText(txt, s, (10, 24 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    return np.vstack([top, txt, plot(ep, k, w)])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="demo dir (with episodes/*.npz, or *.npz directly) or one .npz")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed, 1 = real time")
    ap.add_argument("--start", type=int, default=0, help="first episode index")
    ap.add_argument("--paused", action="store_true")
    a = ap.parse_args()
    if a.path.endswith(".npz"):
        files = [a.path]
    else:
        files = sorted(glob.glob(os.path.join(a.path, "episodes", "*.npz"))) or sorted(glob.glob(os.path.join(a.path, "*.npz")))
    if not files:
        sys.exit(f"no .npz under {a.path}")
    idx, speed, playing = min(a.start, len(files) - 1), a.speed, not a.paused
    cv2.namedWindow(WIN)
    state = {"k": 0, "seek": None}
    cv2.createTrackbar("frame", WIN, 0, 1, lambda v: state.__setitem__("seek", v))
    carry_t = 0.0                                         # time into the episode kept across n / p
    while True:
        ep = load(files[idx])
        n = len(ep["frames"])
        k0 = int(np.argmin(np.abs((ep["ts"] - ep["ts"][0]) - carry_t)))   # the closest frame to carry_t
        state["k"], state["seek"] = k0, None
        cv2.setTrackbarMax("frame", WIN, n - 1)
        cv2.setTrackbarPos("frame", WIN, k0)
        state["seek"] = None
        t_play, k_play = time.monotonic(), state["k"]       # play on from the carried frame
        nxt = None
        while nxt is None:
            if state["seek"] is not None and state["seek"] != state["k"]:
                state["k"], t_play, k_play = state["seek"], time.monotonic(), state["seek"]
            state["seek"] = None
            if playing:                                   # real frame timing, scaled by speed
                el = (time.monotonic() - t_play) * speed
                k = k_play
                while k < n - 1 and ep["ts"][k + 1] - ep["ts"][k_play] <= el:
                    k += 1
                if k != state["k"]:
                    state["k"] = k
                    cv2.setTrackbarPos("frame", WIN, k)
                if k >= n - 1:
                    playing = False
            cv2.imshow(WIN, render(ep, state["k"], speed, playing, idx, len(files)))
            key = cv2.waitKey(10) & 0xFF
            if key in (ord("q"), 27):
                cv2.destroyAllWindows()
                return
            if key == ord(" "):
                playing = not playing
                if playing and state["k"] >= n - 1:
                    state["k"] = 0
            elif key in (ord("+"), ord("=")):
                speed = min(16.0, speed * 2)
            elif key in (ord("-"), ord("_")):
                speed = max(0.125, speed / 2)
            elif key == ord("."):
                playing, state["k"] = False, min(n - 1, state["k"] + 1)
            elif key == ord(","):
                playing, state["k"] = False, max(0, state["k"] - 1)
            elif key == ord("r"):
                state["k"], playing = 0, True
            elif key == ord("n") and idx + 1 < len(files):
                nxt = idx + 1
                carry_t = ep["ts"][state["k"]] - ep["ts"][0]
            elif key == ord("p") and idx > 0:
                nxt = idx - 1
                carry_t = ep["ts"][state["k"]] - ep["ts"][0]
            else:
                continue
            if key in (ord("."), ord(","), ord("r")):
                cv2.setTrackbarPos("frame", WIN, state["k"])
            t_play, k_play = time.monotonic(), state["k"]   # re-anchor timing on any change
        idx = nxt                                         # play / pause stays as it was


if __name__ == "__main__":
    main()
