# Efficient AprilTag / ArUco tracking from `/oakd/camera_info`

How to run a low-cost AprilTag (or ArUco) pose tracker in Python + OpenCV against
the OAK-D 1920×1080 stream — **without** running `detectMarkers` on the full
image every frame.

Intrinsics live in `config/oakd_camera_info_1920x1080.yaml` (captured from the
published `/oakd/camera_info`, see [Intrinsics](#intrinsics)).

---

## 1. TL;DR

`detectMarkers` has **no ROI / initial-guess parameter** — but the "patch" is
just the image you hand it. So:

1. Detect once, get `rvec`/`tvec` at full resolution.
2. Next frame: **project the last pose → predicted corner pixels**.
3. Crop a box around those corners, run `detectMarkers` on the **crop only**.
4. Add the crop offset back to the returned corners, feed the **same `K` + `d`**
   to `solvePnP`.
5. On any failed detection, fall back to a full-frame pass (re-acquire).

Cost: thresholding + contour extraction run on a few-hundred-pixel box instead of
2.1 MP → ~1–2 orders of magnitude less work per frame, while pose precision stays
full-resolution (corners are detected at full res inside the ROI).

---

## 2. Why cropping does NOT change the intrinsics

The projection model (after undistortion) is `u = fx·x_n + cx`. Cropping shifts
every pixel by a constant offset; the lens geometry (`fx, fy, cx, cy`, distortion)
does not change.

| Operation | Pixel effect | Effect on `K` |
|---|---|---|
| **Crop** (ROI) | shifts origin by `(x0, y0)` | **none** — add the offset back to the corners |
| **Resize** (downscale) | scales all pixel positions by `s` | multiply `fx, fy, cx, cy` by `s` |

So a corner detected in the ROI, offset back, lands on the exact pixel it would
have had in the full frame → the published `K` is valid verbatim. (`d` is
focal-length-normalized, hence resolution-invariant, so it passes through
unchanged too.)

Equivalently — and mathematically identically — you can subtract the offset from
the principal point instead of moving the corners:

```python
K_crop = K.copy()
K_crop[0, 2] -= x0   # cx
K_crop[1, 2] -= y0   # cy
rvec, tvec = cv2.solvePnP(obj, corners_roi, K_crop, d)
```

Prefer the **offset-the-corners** form: it keeps the published `/oakd/camera_info`
the single source of truth and avoids a second, easy-to-stale `K`.

> Resize would be a different story: if you ever downscale the *detection* image,
> either scale `K` by the same ratio or multiply the corners back up by `1/ratio`
> before `solvePnP` (equivalent — pick one).

---

## 3. Reference implementation

Runnable against the OAK-D 1080p stream (or any `cv2.VideoCapture`/GStreamer
source). AprilTag via OpenCV's `aruco` module (no extra dependency); swap the
dictionary line for classic ArUco.

```python
#!/usr/bin/env python3
"""Pose-prior AprilTag tracker: ROI detect from last-pose prediction."""
import cv2
import numpy as np

# ── Intrinsics from config/oakd_camera_info_1920x1080.yaml ──────────────
fs = cv2.FileStorage("config/oakd_camera_info_1920x1080.yaml", cv2.FILE_STORAGE_READ)
K = fs.getNode("camera_matrix").mat()
d = fs.getNode("distortion_coefficients").mat().reshape(-1)
W = int(fs.getNode("image_width").real())
H = int(fs.getNode("image_height").real())
fs.release()

# ── Detector: AprilTag 36h11 (or DICT_4X4_50 / DICT_6X6_250 for ArUco) ──
detector = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
    cv2.aruco.DetectorParameters())

TARGET_ID = 0
MARKER_M  = 0.100          # real marker side length, metres
obj = np.array([[0, 0, 0], [MARKER_M, 0, 0],
                [MARKER_M, MARKER_M, 0], [0, MARKER_M, 0]], np.float32)
# detectMarkers corner order is [TL, TR, BR, BL] — matches obj above.

# ROI margin (px). Must exceed worst-case inter-frame marker motion AND
# adaptiveThreshWinSizeMax (default 23). For a fixed camera this is a
# bounded number you measure once; small margin = cheap frames forever.
MARGIN = 80
REACQUIRE_EVERY = 300      # periodic full-frame pass, frames

last_rvec = last_tvec = None
frame = 0

def full_detect(img):
    """Full-frame detection + pose. Returns (rvec, tvec) or None."""
    corners, ids, _ = detector.detectMarkers(img)
    if ids is None or not (ids == TARGET_ID).any():
        return None
    i = int(np.argwhere(ids == TARGET_ID)[0])
    rvec, tvec = cv2.solvePnP(obj, corners[i][0], K, d)
    return rvec, tvec

# ── main loop (img = grayscale 1920x1080) ────────────────────────────────
while True:
    img = ...   # your frame source
    rvec = tvec = None
    use_roi = last_rvec is not None and (frame % REACQUIRE_EVERY != 0)

    if use_roi:
        # 1. Predict corner pixels from the last pose
        pred, _ = cv2.projectPoints(obj, last_rvec, last_tvec, K, d)
        pred = pred.reshape(-1, 2)

        # 2. Crop a box around the prediction
        x0 = int(pred[:, 0].min()) - MARGIN; x1 = int(pred[:, 0].max()) + MARGIN
        y0 = int(pred[:, 1].min()) - MARGIN; y1 = int(pred[:, 1].max()) + MARGIN
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(W, x1), min(H, y1)
        roi = img[y0:y1, x0:x1]

        # 3. Detect on the patch only
        corners, ids, _ = detector.detectMarkers(roi)
        if ids is not None and (ids == TARGET_ID).any():
            i = int(np.argwhere(ids == TARGET_ID)[0])
            c = corners[i][0] + np.float32([x0, y0])   # offset back to full frame
            # 4. solvePnP with the prior as initial guess (ITERATIVE default)
            rvec, tvec = cv2.solvePnP(obj, c, K, d, last_rvec, last_tvec, True)

    if rvec is None:                     # first frame / lost track → full pass
        pose = full_detect(img)
        if pose is not None:
            rvec, tvec = pose

    if rvec is not None:
        last_rvec, last_tvec = rvec, tvec
        # publish / use tvec (metres), rvec (Rodrigues) ...
    frame += 1
```

Key behaviors:

- **First frame** and any **track loss** hit the full-frame path — the only time
  you pay full 2.1 MP cost.
- **`REACQUIRE_EVERY`** keeps a slow full-frame heartbeat so a slowly-drifting
  camera (or marker) re-anchors even if nothing was ever lost.
- **`useExtrinsicGuess=True`** (`..., last_rvec, last_tvec, True`) gives
  `solvePnP` the pose prior too — ITERATIVE mode seeds from it, so the *pose*
  itself gets a better initial guess, not just the search region.
- If the marker is **moving** (e.g. an end-effector target), run a constant-
  velocity filter on `[tvec, rvec]` and predict *from the filtered pose* each
  frame, then fuse the ROI measurement. The camera-fixed case is the same code
  with the velocity model set to zero.

---

## 4. API facts that matter (pitfalls)

1. **No ROI parameter exists.** Cropping *is* the patch API. The ROI is a numpy
   slice (`gray[y0:y1, x0:x1]`) — a view, not a copy — so there's no per-frame
   `cv2.resize` or memcpy tax.

2. **Modern `ArucoDetector` doesn't take `cameraMatrix`/`distCoeffs` at all.**
   The *legacy* `cv2.aruco.detectMarkers(img, dict, params, camMatrix, dist)`
   signature does — and when given, it runs a **full-image `cv2.undistort`**
   before detecting. That (a) destroys the ROI saving and (b) returns corners in
   *undistorted* coordinates, breaking the `+ (x0, y0)` bookkeeping. Use the
   `ArucoDetector` class and let `solvePnP` absorb `d`.

3. **Adaptive-threshold border truncation.** The threshold at a pixel uses a
   window around it; pixels near the crop edge get a truncated window. Harmless
   as long as `MARGIN >= adaptiveThreshWinSizeMax` (default 23). At 80 px you
   never notice.

4. **`minMarkerPerimeterRate` / `maxMarkerPerimeterRate` are relative to the
   image's max dimension.** On a crop, allowed marker-perimeter ratios scale down
   with the ROI size, so cropping only makes detection *more* lenient — except at
   the top end: keep the marker from nearly filling the ROI (else
   `perimeter / max(ROI dims) > 4.0` and it gets rejected). A margin larger than
   the marker's on-screen extent avoids this.

5. **AprilTag specifics.** `DICT_APRILTAG_36h11` etc. run through the same
   detector with the AprilTag quad solver; OpenCV applies the AprilTag corner
   refinement automatically, so returned corners are subpixel and usable directly.
   AprilTag decimation (`aprilTagQuadDecimate`) and
   `aprilTagMinClusterPixels` are tuned per full-frame images — they behave fine
   on a crop, just be aware they're also relative to the ROI.

6. **Don't hand the pose into the measurement.** The predicted corners are the
   *search prior*; the corners the detector returns are still the *measurement*.
   Keep that separation or the filter starts trusting its own prediction.

---

## 5. Cost

Stock `detectMarkers` on a modern x86 host, per frame:

| Input | Default params | Tightened (`adaptiveThreshWinSizeStep=10`, no refine) |
|---|---|---|
| 1280×720 | ~8–15 ms | ~4–8 ms |
| 1920×1080 | ~15–40 ms | ~8–15 ms |
| ROI box ~200×200 px | ~0.1–0.5 ms | ~0.05–0.2 ms |

ROI detection is ~2–3 orders of magnitude cheaper than full-frame 1080p, and it
keeps **full-resolution** corners (pose precision is not traded away, unlike
downscaling the whole detector to 720p).

Tuning levers, in cost order:

1. `adaptiveThreshWinSizeStep` larger → fewer threshold passes (biggest win).
2. ROI from pose prediction (this document) → don't threshold the full frame.
3. `cornerRefinementMethod` — `NONE` (default) is cheapest; add
   `CORNER_REFINE_SUBPIX` only if you need max precision and can afford it.

---

## 6. Intrinsics

Two resolution-specific intrinsics files live here (OpenCV FileStorage
format, loadable directly with `cv2.FileStorage`):

- `config/oakd_camera_info_1920x1080.yaml` — **live-captured** values from
  `/oakd/camera_info` (DOMAIN_ID=30), published by the 1080p streamer
  (`depthai_udp_streamer`).
- `config/oakd_camera_info_1280x720.yaml` — **derived** for the 720p streamer
  (`depthai_720p_udp_streamer`). The 720p feed is the same 16:9 ISP center-crop
  of the IMX378 sensor scaled by 1280/1920 = 2/3, so every K entry is the 1080p
  value × 2/3 exactly (DepthAI's `getCameraIntrinsics` applies the same uniform
  rescale); `d` is unchanged (focal-length-normalized).

- `distortion_model: rational_polynomial` — 8 coefficients in DepthAI EEPROM
  order `[k1,k2,p1,p2,k3,k4,k5,k6]`, focal-length-normalized.
- The streamer republishes **per-resolution** intrinsics (K rescaled for the
  actual streamed size). To re-capture for either streamer, echo the topic
  while it runs and save:

  ```bash
  source /opt/ros/jazzy/setup.bash
  ros2 topic echo --once /oakd/camera_info
  ```

  (Or subscribe to `/oakd/camera_info` in the node itself and rebuild `K`/`d`
  from each message — then the YAML is only a fallback for offline work.)

- ArUco/OpenCV consumers never look at the `distortion_model` *string* — they
  read `d` straight through the model, so `solvePnP` / `undistort` just work.
  The string only matters to ROS `image_proc`, which recognizes
  `rational_polynomial` explicitly.

---

## 7. Standalone multi-camera capture (2× RealSense D405 + OAK-D)

> **Where things live.** The intrinsics config (`config/oakd_camera_info_1920x1080.yaml`)
> and **this AprilTag/ArUco tracker guide live in the OAK-D package**
> (`ai_worker/ffw_depthai/`). The two RealSense D405s are a separate module —
> `ai_worker/ffw_stream/` (`config/camera_left_config.yaml` serial
> `230422272589`, `config/camera_right_config.yaml` serial `230422271116`).

The script below is fully standalone. It opens the two RealSense D405s **directly
over USB** (pyrealsense2) and receives the OAK-D stream over the **UDP link it
actually uses** (GStreamer RTP/H264 — same pipeline as
`ai_worker/ffw_stream/scripts/udp_receiver.py`). Each source is independent:
delete the ones you don't need, and the lazy imports mean missing
pyrealsense2/GStreamer won't break the other halves.

Requirements: `pip install pyrealsense2 numpy opencv-python`, plus system
`gstreamer-1.0` + `gst-plugins-good` + `python3-gi` for the OAK-D half.

```python
#!/usr/bin/env python3
"""Standalone capture: RealSense D405 left/right (color+depth+IR) + OAK-D."""
import cv2
import numpy as np
import threading

# ── RealSense D405s (serials from ffw_stream/config/camera_*_config.yaml) ──
LEFT_SERIAL, RIGHT_SERIAL = "230422272589", "230422271116"
RS_DEPTH = (480, 270, 30)   # Z16  | fits USB 2.0 at 30fps (per configs)
RS_COLOR = (640, 480, 30)   # BGR8 | USB 2.0: drop IR or lower res if it errors
RS_IR    = (480, 270, 30)   # Y8   | D405 needs IR index 1 (not 0)

def open_realsense(serial, with_ir=True):
    """Return an rs.pipeline for one device, or None if it isn't attached."""
    import pyrealsense2 as rs
    ctx = rs.context()
    present = [d.get_info(rs.camera_info.serial_number) for d in ctx.query_devices()]
    if serial not in present:
        print(f"[RealSense] {serial} not present ({present}) — skipping")
        return None
    cfg = rs.config()
    cfg.enable_device(serial)
    w, h, fps = RS_DEPTH; cfg.enable_stream(rs.stream.depth, 0, w, h, rs.format.z16, fps)
    w, h, fps = RS_COLOR; cfg.enable_stream(rs.stream.color, 0, w, h, rs.format.bgr8, fps)
    if with_ir:
        w, h, fps = RS_IR; cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, fps)
    p = rs.pipeline(ctx)
    try:
        p.start(cfg)
    except Exception as e:
        print(f"[RealSense] {serial} start failed: {e}")
        return None
    print(f"[RealSense] {serial} streaming depth{RS_DEPTH} color{RS_COLOR} ir{RS_IR}")
    return p

# ── OAK-D UDP H264 receiver ────────────────────────────────────────────────
# port 9100 = 1080p streamer (K in oakd_camera_info_1920x1080.yaml),
# port 9110 = 720p streamer (K in oakd_camera_info_1280x720.yaml — see section 6).
def open_oakd(port=9100):
    """Return {'frame': BGR, 'lock'} updated by a GStreamer appsink thread."""
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstApp', '1.0')
    from gi.repository import Gst, GstApp   # noqa: F401  (loads appsink API)
    Gst.init(None)
    share = {"frame": None, "lock": threading.Lock()}
    pipe_str = (f"udpsrc port={port} ! application/x-rtp, media=video, "
                f"clock-rate=90000, encoding-name=H264 ! "
                f"rtph264depay ! decodebin ! videoconvert ! "
                f"video/x-raw,format=BGR ! "
                f"appsink name=sink drop=true max-buffers=1")
    pipe = Gst.parse_launch(pipe_str)
    sink = pipe.get_by_name("sink")
    sink.set_property("emit-signals", True)
    def on_sample(s):
        sample = s.emit("pull-sample")
        if sample is not None:
            buf = sample.get_buffer()
            caps = sample.get_caps().get_structure(0)
            w = caps.get_value("width"); h = caps.get_value("height")
            ok, info = buf.map(Gst.MapFlags.READ)
            if ok:
                img = np.ndarray((h, w, 3), buffer=info.data, dtype=np.uint8).copy()
                with share["lock"]:
                    share["frame"] = img
                buf.unmap(info)
        return Gst.FlowReturn.OK
    sink.connect("new-sample", on_sample)
    pipe.set_state(Gst.State.PLAYING)
    print(f"[OAK-D] listening udp port {port} (9100 -> 1080p, matches the YAML K)")
    return share

# ── Open everything ────────────────────────────────────────────────────────
rs_pipes = {side: open_realsense(ser)
            for side, ser in (("left", LEFT_SERIAL), ("right", RIGHT_SERIAL))}
oakd = open_oakd(port=9100)

frames = {}   # side -> {"color":.., "depth":.., "ir":..}

while (cv2.waitKey(1) & 0xFF) != ord('q'):
    for side, p in rs_pipes.items():
        if p is None:
            continue
        fs = p.try_wait_for_frames(timeout_ms=200)
        if not fs:
            continue
        c = fs.get_color_frame()
        z = fs.get_depth_frame()
        ir = fs.get_infrared_frame()
        if c is not None:
            frames.setdefault(side, {})["color"] = np.asanyarray(c.get_data())
            cv2.imshow(f"{side} color", frames[side]["color"])
        if z is not None:
            depth = np.asanyarray(z.get_data())           # Z16, units = 0.001 m
            d = np.clip(depth.astype(np.float32) * 0.001, 0.0, 1.0)
            cv2.imshow(f"{side} depth",
                       cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_JET))
        if ir is not None:
            frames.setdefault(side, {})["ir"] = np.asanyarray(ir.get_data())
            cv2.imshow(f"{side} IR", frames[side]["ir"])

    with oakd["lock"]:
        img = oakd["frame"]
    if img is not None:
        cv2.imshow("OAK-D 1080p", img)
        # feed this frame into the section-3 tracker:
        #   gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #   rvec, tvec = ... (ROI detect with K, d from the YAML)

for p in rs_pipes.values():
    if p is not None:
        p.stop()
cv2.destroyAllWindows()
```

Notes on the RealSense half:

- **Two devices, one `rs.context()`** — each `rs.pipeline(ctx)` is started with
  `enable_device(serial)`, so the left/right D405s run concurrently.
- **D405 IR index 1** — the configs flag this (`ir_index: 1`); index 0 would fail
  on this device.
- **USB 2.0 bandwidth** — depth+color+IR together at 30 fps may exceed the bus.
  If `start` fails or a stream stalls, drop IR first (`with_ir=False`) or lower
  `RS_COLOR` to 480×270.
- Depth units: raw Z16 × `0.001` → metres (matches `udp_receiver.py`).

And the OAK-D half: the port must match the intrinsics you feed the tracker —
**9100 = 1080p** → the OAK-D package's `config/oakd_camera_info_1920x1080.yaml`;
**9110 = 720p** → `config/oakd_camera_info_1280x720.yaml` (see section 6).
