#!/usr/bin/env python3
"""Teach peg waypoints relative to the hole: you drive the peg with the right SpaceMouse, the tool
records where it is IN THE HOLE'S FRAME, so a scripted path (e.g. the reset's seat nudge) can be
replayed at any hole pose (peg_hole_serl_server.py --seat-path FILE).

The left (hole) arm holds where it is; the right arm is yours (SpaceMouse, joy_hand). The window
shows, live, the peg tool point against the hole: height above the modelled block top, the offset
across the hole axis -- also in ROBOT directions (base +x / +y projected onto the hole's plane, so
"front" / "left" can be checked) -- the tilt, and how hard you press (the commanded pose -- the IK
solution -- below the measured one).

Keys (window "peg-hole waypoints"):
  r   record a waypoint, reached by a normal move
  p   record a PRESS / SLIDE waypoint: replayed slowly, in contact (the commanded pose is kept, so
      pressing deeper than the block top replays as pressing that hard)
  d   delete the last waypoint        s   save        q   quit (asks to save if unsaved)

Saved: <out>/<name>.json -- every waypoint as the peg tool pose relative to the hole tool frame,
commanded (rel_cmd, what replay drives to) and measured (rel_meas), 4x4, plus a readable summary.

  source ROS; .venv/bin/python peg_hole_waypoint_gui.py --allow-motion --name back_edge_slide
"""
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rclpy.signals
from rclpy.executors import SingleThreadedExecutor

import peg_hole_random as phr
import peg_hole_teach as pht

HERE = Path(__file__).resolve().parent
WIN = "peg-hole waypoints"


def rel_summary(H, rel):
    """Readable numbers of a peg-tool pose rel (in the hole tool frame H)."""
    p = rel[:3, 3]
    lat_base = H[:3, 1] * p[1] + H[:3, 2] * p[2]                      # the across-axis offset, in base_link
    tilt = float(np.degrees(np.arccos(np.clip(rel[0, 0], -1.0, 1.0))))
    return {"above_top_mm": (p[0] - pht.TOP) * 1000, "y_mm": p[1] * 1000, "z_mm": p[2] * 1000,
            "base_x_mm": lat_base[0] * 1000, "base_y_mm": lat_base[1] * 1000,
            "lateral_mm": float(np.hypot(p[1], p[2]) * 1000), "tilt_deg": tilt}


class Gui:
    def __init__(self, io, t, a):
        self.io, self.t, self.a = io, t, a
        self.map = np.linalg.inv(io.ee("right")) @ io.latest_site("right")    # TF EE -> IK site (rigid)
        self.wps, self.dirty, self.msg = [], False, ""
        self.path = Path(a.out) / f"{a.name}.json"

    def poses(self):
        """(hole tool frame H, peg tool rel measured, peg tool rel commanded), all live."""
        H = self.io.ee("left") @ pht.HOLE_TOOL
        Hi = np.linalg.inv(H)
        meas = Hi @ self.io.ee("right") @ pht.PEG_TOOL
        cmd = Hi @ self.io.latest_site("right") @ np.linalg.inv(self.map) @ pht.PEG_TOOL
        return H, meas, cmd

    def record(self, kind):
        H, meas, cmd = self.poses()
        w = {"i": len(self.wps) + 1, "kind": kind, "rel_cmd": cmd.tolist(), "rel_meas": meas.tolist(),
             "summary_cmd": rel_summary(H, cmd), "summary_meas": rel_summary(H, meas)}
        self.wps.append(w)
        self.dirty = True
        s = w["summary_cmd"]
        self.msg = (f"recorded #{w['i']} {kind}: {s['above_top_mm']:+.1f} mm above the top, base x {s['base_x_mm']:+.1f} "
                    f"y {s['base_y_mm']:+.1f} mm, press {(w['summary_meas']['above_top_mm'] - s['above_top_mm']):.1f} mm")
        print(self.msg, flush=True)

    def save(self):
        H, _, _ = self.poses()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps({
            "time": datetime.now().isoformat(timespec="seconds"), "frame": "peg tool pose in the hole tool frame "
            "(x = insertion axis up, from the hole tool origin; TOP = the modelled block top on x)",
            "TOP_m": pht.TOP, "hole_ee_tf": self.io.ee("left").tolist(), "lift": self.a.lift,
            "waypoints": self.wps}, indent=1) + "\n")
        self.dirty = False
        self.msg = f"saved {len(self.wps)} waypoints -> {self.path}"
        print(self.msg, flush=True)

    def draw(self):
        H, meas, cmd = self.poses()
        sm, sc = rel_summary(H, meas), rel_summary(H, cmd)
        lines = [f"peg vs hole (measured): {sm['above_top_mm']:+6.1f} mm above the modelled top   lateral {sm['lateral_mm']:5.1f} mm"
                 f"   tilt {sm['tilt_deg']:4.1f} deg",
                 f"  across the axis in robot directions: base x {sm['base_x_mm']:+6.1f} mm   base y (left +) {sm['base_y_mm']:+6.1f} mm",
                 f"  hole frame: y {sm['y_mm']:+6.1f}  z {sm['z_mm']:+6.1f} mm",
                 f"press (commanded below measured): {sm['above_top_mm'] - sc['above_top_mm']:+5.1f} mm",
                 f"waypoints: {len(self.wps)}" + ("  [unsaved]" if self.dirty else "") + "   " +
                 "  ".join(f"#{w['i']}{'P' if w['kind'] == 'press' else ''}" for w in self.wps[-8:]),
                 self.msg,
                 "r: record   p: record press/slide   d: delete last   s: save   q: quit"]
        img = np.full((30 + 30 * len(lines), 1000, 3), 30, np.uint8)
        for i, s in enumerate(lines):
            cv2.putText(img, s, (10, 30 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.imshow(WIN, img)

    def run(self):
        last = 0.0
        while True:
            self.t.st.tick(None)                          # keeps the hole arm's goal streaming
            if time.monotonic() - last < 0.05:
                continue
            last = time.monotonic()
            self.draw()
            k = cv2.waitKey(1) & 0xFF
            if k == ord("r"):
                self.record("move")
            elif k == ord("p"):
                self.record("press")
            elif k == ord("d") and self.wps:
                self.wps.pop()
                self.dirty, self.msg = True, f"deleted; {len(self.wps)} left"
            elif k == ord("s"):
                self.save()
            elif k in (ord("q"), 27):
                if self.dirty and self.wps:
                    self.msg, self.dirty = "unsaved waypoints: s to save, q again to quit", False
                    continue
                return


def main():
    ap = pht.build_parser(__doc__, default_out=str(HERE / "recordings" / "peg_hole_waypoints"))
    ap.add_argument("--name", default=datetime.now().strftime("waypoints_%Y%m%d_%H%M%S"))
    a = ap.parse_args()
    if not a.allow_motion:
        ap.error("pass --allow-motion")
    pht.apply_setup(a)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    io = pht.TeachIo(True)
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    threading.Thread(target=ex.spin, daemon=True).start()
    t = None
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0 and not all(io.ee(s) is not None and io.latest_site(s) is not None
                                                        for s in ("left", "right")):
            time.sleep(0.1)
        t = pht.Teach(io, a)
        t.set_tool_model(a.tool_model)
        t.st.arms["left"] = phr.Arm("left", io.ee("left"), io.latest_site("left"))   # the hole holds
        io.set_hold_side("left", True)
        t.give_right()                                                               # the peg is yours
        print("hole held; right arm on the SpaceMouse. Keys in the window: r / p record, d, s, q", flush=True)
        Gui(io, t, a).run()
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if t is not None:
            t.summary.close()
        io.set_hold(False)
        print("both arms released to the SpaceMouse", flush=True)
        cv2.destroyAllWindows()
        time.sleep(0.2)
        ex.shutdown()
        io.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
