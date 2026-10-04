#!/usr/bin/env python3
"""Manual peg-hole alignment over a pose grid: you align by eye, the tool records.

For each of --grid-n x --grid-n hole poses across the insertion axis (+- --extent,
serpentine order), with a random height along the axis (--height range) and a random
tilt about the two cross axes (+- --tilt-range deg), all reach-checked:
  1. the peg is lifted straight up the hole axis to CLEAR (never sideways near the block),
     the hole moves to the pose, and the peg comes to rest aligned 1.5 mm above ON_TOP;
  2. you drive the peg with the right SpaceMouse (translation only); every control tick
     the peg's orientation is re-aligned to the live hole axis about the peg tool point,
     so the bottom stays where you put it and only the angle follows the hole;
  3. Enter records the alignment (peg tool pose in the hole tool frame, commanded and
     measured: height on the axis, offset across it, residual tilt) and moves on.
Keys (window "peg-hole align"): Enter = record + next, s = skip pose, r = redo pose
(re-reset), space = hold (stop moving), q = quit. The hole is watched: pushed sideways
more than 6 mm -> hold.

Results -> <out>/<session>/align.json after every pose.

  source ROS; .venv/bin/python peg_hole_align_gui.py --allow-motion
"""
import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import rclpy.signals
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as Rot

import peg_hole_random as phr
import peg_hole_stroke as phs
import peg_hole_teach as pht

HERE = Path(__file__).resolve().parent
HOLE_SHIFT_MAX = 0.006
HOVER = 0.0015                                  # m above ON_TOP where each pose starts


class Hold(Exception):
    pass


class AlignGui:
    def __init__(self, t, a):
        self.t, self.a, self.io = t, a, t.io
        self.rng = np.random.default_rng(a.seed if a.seed is not None else 0)
        self.key, self.msg, self.pose_info = None, "", ""
        self.watch, self.holding = None, False
        self.results, self.samples = [], {}
        t.st.on_tick = self.on_tick                    # recorder + our window/keys on every control tick

    # --- window ----------------------------------------------------------------------------
    def rel(self):
        """Peg tool pose in the live hole tool frame: (height above ON_TOP, offset y, z, tilt deg)."""
        H = self.io.ee("left") @ pht.HOLE_TOOL
        P = np.linalg.inv(H) @ self.io.ee("right") @ pht.PEG_TOOL
        tilt = np.degrees(np.arccos(np.clip(P[0, 0], -1.0, 1.0)))
        return P[0, 3] - pht.TOP, P[1, 3], P[2, 3], tilt

    def on_tick(self):
        import cv2
        self.t.rec.sample(self.t.st)
        h, y, z, tilt = self.rel()
        lines = [self.pose_info,
                 f"peg vs hole: height {h * 1000:+6.2f} mm over ON_TOP   offset y {y * 1000:+5.2f}  z {z * 1000:+5.2f} mm"
                 f"   tilt {tilt:4.2f} deg",
                 f"recorded {len(self.results)} poses",
                 self.msg,
                 "SpaceMouse: move the peg (orientation follows the hole)",
                 "Enter: record + next   s: skip   r: redo pose   space: hold   q: quit"]
        img = np.full((30 + 30 * len(lines), 1000, 3), 30, np.uint8)
        for i, s in enumerate(lines):
            cv2.putText(img, s, (10, 30 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        cv2.imshow("peg-hole align", img)
        k = cv2.waitKey(1) & 0xFF
        if k in (10, 13):
            self.key = "enter"
        elif k != 255:
            self.key = chr(k)
        if self.key == " ":
            self.key = None
            if not self.holding:
                raise Hold("space pressed")
        if self.watch is not None:
            d = (self.io.ee("left") @ pht.HOLE_TOOL)[:3, 3] - self.watch[0]
            if np.linalg.norm(d - (d @ self.watch[1]) * self.watch[1]) > HOLE_SHIFT_MAX:
                self.watch = None
                raise Hold(f"hole pushed > {HOLE_SHIFT_MAX * 1000:.0f} mm sideways")

    def watch_hole(self):
        H = self.io.ee("left") @ pht.HOLE_TOOL
        self.watch = (H[:3, 3].copy(), H[:3, 0].copy())

    # --- poses -------------------------------------------------------------------------------
    def poses(self):
        a = self.a
        H0 = self.t.L_center @ pht.HOLE_TOOL
        g = np.linspace(-a.extent, a.extent, a.grid_n)
        out = []
        for i, gy in enumerate(g):
            for gz in (g if i % 2 == 0 else g[::-1]):
                out.append((gy, gz))
        return H0, out

    def sample_pose(self, H0, gy, gz):
        """Hole EE pose at grid (gy, gz) with a random height and tilt (reach-checked), or None."""
        a = self.a
        roll = Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix()
        hover = pht.HOLE_TOOL @ phs.T_from([pht.TOP + HOVER, 0, 0], roll) @ np.linalg.inv(pht.PEG_TOOL)
        for _ in range(20):
            hx = self.rng.uniform(*a.height)
            tilt = self.rng.uniform(-a.tilt_hole, a.tilt_hole, 2)
            H = H0.copy()
            H[:3, 3] = H0[:3, 3] + H0[:3, :3] @ np.array([hx, gy, gz])
            H[:3, :3] = H0[:3, :3] @ Rot.from_euler("yz", tilt, degrees=True).as_matrix()   # about the cross axes
            hole_ee = H @ np.linalg.inv(pht.HOLE_TOOL)
            ok, why, mg, j = self.t.check_sample(hole_ee, hover)
            if ok:
                return hole_ee, hover, hx, tilt
            print(f"  sample rejected: {why}")
        return None

    def retract(self):
        """Peg straight up the hole axis to CLEAR (keeps its offset): never sideways near the block."""
        self.t.take_right()
        H = self.io.ee("left") @ pht.HOLE_TOOL
        T_peg = self.io.ee("right") @ pht.PEG_TOOL
        rise = pht.TOP + self.a.clear - (np.linalg.inv(H) @ np.r_[T_peg[:3, 3], 1.0])[0]
        if rise > 0.0:
            T_up = T_peg.copy()
            T_up[:3, 3] += rise * H[:3, 0]
            self.watch_hole()
            try:
                self.t.move_right(T_up @ np.linalg.inv(pht.PEG_TOOL), self.a.speed, "reset_out")
            finally:
                self.watch = None

    def align_loop(self):
        """SpaceMouse translation, orientation re-aligned to the live hole axis every tick about
        the peg tool point (teach manual mode). -> 'enter' / 's' / 'r' / 'q'."""
        t = self.t
        t.manual_trans = (t.io.ee("left") @ pht.HOLE_TOOL)[:3, 0] * HOVER     # start where the reset left it
        self.watch_hole()
        self.key = None
        try:
            while self.key not in ("enter", "s", "r", "q"):
                t.manual_tick()
        finally:
            self.watch = None
        return self.key

    def record(self, n, gy, gz, hx, tilt):
        h, y, z, tl = self.rel()
        off = self.t.taught_offset()
        r = {"pose": n, "grid_m": [float(gy), float(gz)], "hole_height_m": float(hx), "hole_tilt_deg": tilt.tolist(),
             "hole_ee_tf": self.io.ee("left").tolist(), "peg_ee_tf": self.io.ee("right").tolist(),
             "measured": {"height_mm": h * 1000, "y_mm": y * 1000, "z_mm": z * 1000, "tilt_deg": tl},
             "commanded_offset_peg_tool": off.tolist(),
             "commanded_mm": {"x": off[0, 3] * 1000, "y": off[1, 3] * 1000, "z": off[2, 3] * 1000}}
        self.results.append(r)
        with open(os.path.join(self.t.out, "align.json"), "w") as f:
            json.dump({"time": datetime.now().isoformat(timespec="seconds"), "lift": self.a.lift,
                       "grid_n": self.a.grid_n, "extent_m": self.a.extent, "height_m": self.a.height,
                       "tilt_deg": self.a.tilt_hole, "results": self.results}, f, indent=1)
        print(f"  recorded pose {n}: offset y {y * 1000:+.2f} z {z * 1000:+.2f} mm, height {h * 1000:+.2f} mm, "
              f"tilt {tl:.2f} deg")

    def run(self):
        H0, grid = self.poses()
        n = 0
        while n < len(grid):
            gy, gz = grid[n]
            if n not in self.samples:                     # r (redo) repeats the same height / tilt
                self.samples[n] = self.sample_pose(H0, gy, gz)
            s = self.samples[n]
            if s is None:
                print(f"pose {n + 1}: no reachable height/tilt, skipped")
                n += 1
                continue
            hole_ee, hover, hx, tilt = s
            self.pose_info = (f"pose {n + 1}/{len(grid)}: grid y {gy * 100:+.1f} z {gz * 100:+.1f} cm, "
                              f"height {hx * 1000:+.1f} mm, tilt {tilt[0]:+.1f} {tilt[1]:+.1f} deg")
            print(self.pose_info)
            try:
                self.msg = "moving to the pose..."
                self.retract()
                if not self.t.hw_reset((hole_ee, hover)):
                    print("  reset failed; holding (r = retry this pose, q = quit)")
                    raise Hold("reset failed")
                self.msg = "align the peg by eye, then Enter"
                k = self.align_loop()
            except (Hold, phr.LagTrip) as e:
                self.msg = f"HOLD: {e} -- r: redo pose, s: skip, q: quit"
                print(f"  hold: {e}")
                self.key, self.holding = None, True
                try:
                    while self.key not in ("r", "s", "q"):
                        self.t.st.tick(None)
                finally:
                    self.holding = False
                k = self.key
            if k == "enter":
                self.record(n + 1, gy, gz, hx, tilt)
                n += 1
            elif k == "s":
                n += 1
            elif k == "q":
                break
        print(f"done: {len(self.results)} poses recorded -> {os.path.join(self.t.out, 'align.json')}")


def main():
    ap = pht.build_parser(__doc__, default_out=str(HERE / "recordings" / "peg_hole_align"))
    ap.add_argument("--grid-n", type=int, default=5)
    ap.add_argument("--extent", type=float, default=0.04, help="m, grid spans +- this across the axis")
    ap.add_argument("--height", type=float, nargs=2, default=[-0.02, 0.0], help="m, random hole height along the axis")
    ap.add_argument("--hover", type=float, default=HOVER, help="m above ON_TOP where each pose starts (default 1.5 mm)")
    ap.add_argument("--tilt-range", dest="tilt_hole", type=float, default=3.0,
                    help="deg, random hole tilt about each cross axis")
    a = ap.parse_args()
    pht.apply_setup(a)
    globals()["HOVER"] = a.hover
    if not a.allow_motion:
        ap.error("pass --allow-motion")
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    io = pht.TeachIo(True)
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    t, lift_before = None, None
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0 and not (
                io.raw_js is not None and io.effort is not None
                and all(io.ee(s) is not None and io.latest_site(s) is not None for s in ("left", "right"))):
            time.sleep(0.1)
        lift = dict(zip(io.raw_js[1], io.raw_js[2])).get("lift_joint") if io.raw_js else None
        if lift is None or abs(lift - a.lift) > 0.003:
            print(f"lift is {lift}, not {a.lift:+.3f}: run peg_hole_safe_setup.py first; quitting")
            return
        if a.lock_lift:
            lift_before = io.set_joint_locked("lift_joint", True)
        t = pht.Teach(io, a)
        t.set_tool_model(a.tool_model)
        t.st.arms["left"] = phr.Arm("left", io.ee("left"), io.latest_site("left"))
        io.set_hold_side("left", True)
        t.take_right()
        print(f"recording to {t.out}")
        AlignGui(t, a).run()
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if t is not None:
            print(f"session saved: {t.rec.save_session()}")
            t.summary.close()
        io.set_hold(False)
        print("both arms released to the SpaceMouse")
        if a.lock_lift and lift_before is False:
            io.set_joint_locked("lift_joint", False)
        time.sleep(0.2)
        ex.shutdown()
        spin.join(timeout=2.0)
        io.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
