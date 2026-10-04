#!/usr/bin/env python3
"""Pick the wrist-camera ROIs for the peg-hole policy images (ffw_peg_hole_env/images.py).

Decodes both wrist D405 RGB feeds straight off UDP (left 9001, right 9003; the
ffw_stream receiver must NOT be running -- it would own the ports), shows each
rotated frame with its ROI box and, next to it, the exact out x out image the
policy gets (shown 2x). Saves to ffw_collision_checker/config/peg_hole_roi.json.

Mouse: click / drag in a camera view = select that camera and centre its ROI there.
Keys (window "peg-hole ROI"):
  tab        switch camera            i j k l    nudge 1 px (I J K L: 10 px)
  + / -      ROI size +-32 px         Enter      save
  q / Esc    quit (asks if unsaved)

  .venv/bin/python peg_hole_roi_gui.py                    # ROI 256 -> 128
  .venv/bin/python peg_hole_roi_gui.py --codec mjpeg      # if the streamer runs --mjpeg
"""
import argparse
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "ffw_peg_hole_env"))
sys.path.insert(0, str(HERE.parents[1] / "ffw_il_recorder"))
from ffw_il_recorder import gst_decode  # noqa: E402
from ffw_peg_hole_env import images  # noqa: E402

WIN = "peg-hole ROI"
PREVIEW = 2                                     # policy image shown at 2x
NUDGE = {ord("i"): (0, -1), ord("k"): (0, 1), ord("j"): (-1, 0), ord("l"): (1, 0)}


class RoiGui:
    def __init__(self, a):
        self.a, self.out = a, a.out
        saved = images.load_roi(a.file) or {}
        if saved and saved.get("out") != a.out:
            print(f"saved ROIs are for out={saved.get('out')}, now {a.out}: sizes kept, check them")
        self.roi = {c: saved.get(c) for c in ("left", "right")}
        self.dec = {c: gst_decode.VideoDecoder(c, a.ports[c], codec=a.codec, feed="rs",
                                               rotate=cv2.ROTATE_90_COUNTERCLOCKWISE)
                    for c in ("left", "right")}
        self.active, self.dirty, self.drag = "right", False, False
        self.layout = {}                        # cam -> (x0 on canvas, w, h) of its view
        self.msg = ""

    # --- ROI -------------------------------------------------------------------------------
    def ensure_roi(self, cam, frame):
        h, w = frame.shape[:2]
        r = self.roi[cam]
        if r is None:
            s = min(self.a.size, w, h)
            r = self.roi[cam] = {"port": self.a.ports[cam], "x": (w - s) // 2, "y": (h - s) // 2, "size": s}
            self.dirty = True
        r["x"], r["y"], r["size"] = images.clamp_roi(r["x"], r["y"], r["size"], w, h)
        r["frame_wh"] = [w, h]

    def centre_at(self, cam, px, py):
        r = self.roi[cam]
        if r is None:
            return
        w, h = r["frame_wh"]
        r["x"], r["y"], r["size"] = images.clamp_roi(px - r["size"] // 2, py - r["size"] // 2, r["size"], w, h)
        self.active, self.dirty = cam, True

    def resize(self, step):
        r = self.roi[self.active]
        if r is None:
            return
        cx, cy = r["x"] + r["size"] // 2, r["y"] + r["size"] // 2
        r["size"] = int(max(self.out, r["size"] + step))
        w, h = r["frame_wh"]
        r["x"], r["y"], r["size"] = images.clamp_roi(cx - r["size"] // 2, cy - r["size"] // 2, r["size"], w, h)
        self.dirty = True

    def save(self):
        if any(self.roi[c] is None for c in ("left", "right")):
            self.msg = "not saved: no frame yet from " + ", ".join(c for c in ("left", "right") if self.roi[c] is None)
            return
        data = {"out": self.out, "left": self.roi["left"], "right": self.roi["right"],
                "saved": datetime.now().isoformat(timespec="seconds")}
        Path(self.a.file).write_text(json.dumps(data, indent=1) + "\n")
        self.dirty = False
        self.msg = f"saved {self.a.file}"
        print(self.msg, json.dumps({c: data[c] for c in ("left", "right")}))

    # --- window ----------------------------------------------------------------------------
    def on_mouse(self, ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN:
            self.drag = True
        elif ev == cv2.EVENT_LBUTTONUP:
            self.drag = False
        if ev == cv2.EVENT_LBUTTONDOWN or (ev == cv2.EVENT_MOUSEMOVE and self.drag):
            s = self.a.display_scale
            for cam, (x0, w, h) in self.layout.items():
                if x0 <= x < x0 + w and y < h:
                    self.centre_at(cam, int((x - x0) / s), int(y / s))

    def view(self, cam):
        """(camera view with the ROI box, policy-image preview), both BGR; placeholders if no frame."""
        frame, recv_t = self.dec[cam].peek()
        side = self.out * PREVIEW
        if frame is None:
            ph = np.zeros((480, 360, 3), np.uint8)
            cv2.putText(ph, f"{cam}: waiting for UDP {self.a.ports[cam]}", (10, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            return ph, np.zeros((side, side, 3), np.uint8)
        self.ensure_roi(cam, frame)
        r = self.roi[cam]
        pol = images.crop(frame, r, self.out)[:, :, ::-1]                       # RGB -> BGR to show
        prev = cv2.resize(pol, (side, side), interpolation=cv2.INTER_NEAREST)
        v = frame.copy()
        col = (0, 255, 0) if cam == self.active else (0, 200, 255)
        cv2.rectangle(v, (r["x"], r["y"]), (r["x"] + r["size"] - 1, r["y"] + r["size"] - 1), col, 2)
        if self.a.display_scale != 1.0:
            v = cv2.resize(v, None, fx=self.a.display_scale, fy=self.a.display_scale, interpolation=cv2.INTER_AREA)
        age = time.time() - recv_t
        txt = (f"{cam}{' *' if cam == self.active else ''}  {frame.shape[1]}x{frame.shape[0]}  "
               f"{self.dec[cam].fps:.0f} fps  age {age * 1000:.0f} ms")
        cv2.putText(v, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        cv2.putText(v, f"roi x {r['x']} y {r['y']} size {r['size']} -> {self.out}", (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        return v, prev

    def draw(self):
        tiles, x0 = [], 0
        for cam in ("left", "right"):
            v, prev = self.view(cam)
            self.layout[cam] = (x0, v.shape[1], v.shape[0])
            tiles += [v, prev]
            x0 += v.shape[1] + prev.shape[1]
        H = max(t.shape[0] for t in tiles) + 40
        canvas = np.zeros((H, sum(t.shape[1] for t in tiles), 3), np.uint8)
        x = 0
        for t in tiles:
            canvas[:t.shape[0], x:x + t.shape[1]] = t
            x += t.shape[1]
        foot = ("click/drag: centre ROI  tab: camera  ijkl/IJKL: nudge  +/-: size  Enter: save  q: quit"
                + ("   [unsaved]" if self.dirty else "") + (f"   {self.msg}" if self.msg else ""))
        cv2.putText(canvas, foot, (8, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow(WIN, canvas)

    def run(self):
        for d in self.dec.values():
            d.start()
        cv2.namedWindow(WIN)
        cv2.setMouseCallback(WIN, self.on_mouse)
        try:
            while True:
                self.draw()
                k = cv2.waitKey(30) & 0xFF
                if k == 255:
                    continue
                self.msg = ""
                if k in (ord("q"), 27):
                    if self.dirty and not self.a.yes:
                        self.msg, self.dirty = "unsaved changes: Enter to save, q again to quit", False
                        continue
                    break
                if k == 9:
                    self.active = "left" if self.active == "right" else "right"
                elif k in (13, 10):
                    self.save()
                elif k in (ord("+"), ord("=")):
                    self.resize(32)
                elif k in (ord("-"), ord("_")):
                    self.resize(-32)
                elif k in NUDGE or (k + 32) in NUDGE:
                    (dx, dy), n = (NUDGE[k], 1) if k in NUDGE else (NUDGE[k + 32], 10)
                    r = self.roi[self.active]
                    if r is not None:
                        w, h = r["frame_wh"]
                        r["x"], r["y"], r["size"] = images.clamp_roi(r["x"] + dx * n, r["y"] + dy * n,
                                                                     r["size"], w, h)
                        self.dirty = True
        finally:
            for d in self.dec.values():
                d.stop()
            cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", default=str(images.ROI_FILE))
    ap.add_argument("--size", type=int, default=2 * images.OUT, help="default ROI size, px (new ROIs only)")
    ap.add_argument("--out", type=int, default=images.OUT, help="policy image size, px")
    ap.add_argument("--codec", default="h264", choices=("h264", "mjpeg"))
    ap.add_argument("--left-port", type=int, default=images.PORTS["left"])
    ap.add_argument("--right-port", type=int, default=images.PORTS["right"])
    ap.add_argument("--display-scale", type=float, default=1.5, help="camera view zoom (frames are 270x480)")
    ap.add_argument("--yes", action="store_true", help="quit without asking about unsaved changes")
    a = ap.parse_args()
    a.ports = {"left": a.left_port, "right": a.right_port}
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    RoiGui(a).run()


if __name__ == "__main__":
    main()
