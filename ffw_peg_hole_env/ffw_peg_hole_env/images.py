"""Wrist-camera images for the real robot: ROI crop + downsample to the policy size.

Both wrist D405s stream RGB over RTP/UDP from the Jetson (ffw_stream
realsense_udp_streamer, the "IR" port of each camera: left 9001, right 9003) to
this workstation; ffw_il_recorder/gst_decode.py decodes them (BGR, rotated 90 deg
CCW like the ROS receiver; 270x480 after rotation, ~15 fps). peg_hole_roi_gui.py
picks one square ROI per camera on that decoded, rotated frame and saves it to
ffw_collision_checker/config/peg_hole_roi.json; crop() turns a decoded frame into the out x out RGB uint8 image
the policy sees (default 256 px ROI -> 128 px, 2x2 area downsample).

ROI file:
  {"out": 128,
   "left":  {"port": 9001, "x": .., "y": .., "size": 256, "frame_wh": [w, h]},
   "right": {"port": 9003, ...},
   "saved": "<iso time>"}
x, y = top-left corner of the ROI in the rotated frame, pixels.
"""
import json
from pathlib import Path

import cv2
import numpy as np

CAMS = ("right", "left")
PORTS = {"left": 9001, "right": 9003}
OUT = 128
ROI_FILE = Path(__file__).resolve().parents[2] / "ffw_collision_checker" / "config" / "peg_hole_roi.json"


def clamp_roi(x, y, size, w, h):
    """Keep a size x size box inside a w x h frame (size shrinks to fit if needed)."""
    size = int(min(size, w, h))
    return int(np.clip(x, 0, w - size)), int(np.clip(y, 0, h - size)), size


def crop(frame_bgr, roi, out=OUT):
    """Decoded BGR frame -> (out, out, 3) RGB uint8 from the ROI {'x','y','size'}."""
    h, w = frame_bgr.shape[:2]
    x, y, s = clamp_roi(roi["x"], roi["y"], roi["size"], w, h)
    patch = frame_bgr[y:y + s, x:x + s]
    if s != out:
        patch = cv2.resize(patch, (out, out), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(patch[:, :, ::-1])


def load_roi(path=ROI_FILE):
    """The saved ROI file as a dict, or None if it does not exist yet."""
    path = Path(path)
    return json.loads(path.read_text()) if path.exists() else None


class WristCameras:
    """Both wrist D405 RGB feeds decoded off UDP (ffw_il_recorder/gst_decode.py, rotated CCW like
    the ROS receiver) and cropped with the saved ROIs. The ffw_stream ROS receiver must not run
    (it would own the ports). grab() -> ({cam: (out, out, 3) RGB uint8 or None}, {cam: recv
    time.time() or NaN}) with the newest frame of each camera."""

    def __init__(self, roi=None, codec="h264"):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ffw_il_recorder"))
        from ffw_il_recorder import gst_decode
        self.roi = roi or load_roi()
        if self.roi is None:
            raise FileNotFoundError(f"no ROI file {ROI_FILE}: run peg_hole_roi_gui.py first")
        self.out = int(self.roi["out"])
        self.dec = {c: gst_decode.VideoDecoder(c, self.roi[c]["port"], codec=codec, feed="rs",
                                               rotate=cv2.ROTATE_90_COUNTERCLOCKWISE) for c in CAMS}
        for d in self.dec.values():
            d.start()

    def grab(self):
        imgs, ts = {}, {}
        for c, d in self.dec.items():
            frame, t = d.peek()
            imgs[c] = None if frame is None else crop(frame, self.roi[c], self.out)
            ts[c] = float("nan") if frame is None else t
        return imgs, ts

    def stop(self):
        for d in self.dec.values():
            d.stop()
