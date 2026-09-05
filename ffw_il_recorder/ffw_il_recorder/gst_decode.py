#!/usr/bin/env python3
"""UDP RTP video decode for the IL episode recorder.

One :class:`VideoDecoder` thread per feed. The pipelines are copied **verbatim**
from ``ffw_stream/src/realsense_udp_receiver.cpp``:

* head      (OAK-D 720p, port 9110) — ``oakd720pStreamLoop``
* right RGB (D405 cam1 IR, port 9003) — ``streamLoop(1, "IR")`` with a
  CCW rotate applied after decode, exactly like the receiver does for ``is_rgb``.

Each thread keeps only the newest decoded frame (drop-oldest — the same
behaviour as the ``queue max-size-buffers=1 leaky=downstream`` stage in the
receiver pipelines). The 30 Hz sampler calls :meth:`snapshot`, which reports
whether a *new* frame arrived since the last call.
"""

import logging
import threading
import time

import cv2

log = logging.getLogger("ffw_il_recorder.gst_decode")

# Verbatim from realsense_udp_receiver.cpp oakd720pStreamLoop() MJPEG branch.
_PIPE_OAKD720P_MJPEG = (
    "udpsrc port={port} buffer-size=2147483647 "
    'caps="application/x-rtp, media=video, encoding-name=JPEG, payload=26" ! '
    "rtpjpegdepay ! jpegdec ! videoconvert ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "appsink drop=true sync=false async=false max-buffers=1"
)
# Verbatim oakd720pStreamLoop() H264 branch.
_PIPE_OAKD720P_H264 = (
    "udpsrc port={port} buffer-size=2147483647 "
    'caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264" ! '
    "rtpjitterbuffer latency=20 ! rtph264depay ! decodebin ! videoconvert ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "appsink drop=true sync=false async=false max-buffers=1"
)
# Verbatim from streamLoop() MJPEG branch (RealSense feeds).
_PIPE_RS_MJPEG = (
    "udpsrc port={port} buffer-size=2147483647 "
    'caps="application/x-rtp, media=video, encoding-name=JPEG, payload=26" ! '
    "rtpjpegdepay ! jpegdec ! videoconvert ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "appsink drop=true sync=false"
)
# Verbatim from streamLoop() H264 branch (RealSense feeds).
_PIPE_RS_H264 = (
    "udpsrc port={port} buffer-size=2147483647 "
    'caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264" ! '
    "rtpjitterbuffer latency=10 ! rtph264depay ! decodebin ! videoconvert ! "
    "appsink drop=true sync=false"
)


def build_pipeline(port, codec, feed):
    """Return the verbatim receiver pipeline for this feed.

    ``feed`` is ``'oakd720p'`` (head) or ``'rs'`` (RealSense / right RGB).
    ``codec`` is ``'mjpeg'`` (default) or ``'h264'``.
    """
    if feed == "oakd720p":
        tmpl = _PIPE_OAKD720P_MJPEG if codec == "mjpeg" else _PIPE_OAKD720P_H264
    else:
        tmpl = _PIPE_RS_MJPEG if codec == "mjpeg" else _PIPE_RS_H264
    return tmpl.format(port=int(port))


class VideoDecoder:
    """Decodes one UDP RTP video feed in a dedicated thread.

    Mirrors the receiver decode loop:

    * ``!isOpened()`` -> log warn, sleep 2 s, recreate the capture, retry.
    * empty ``read()`` -> 5 ms nap, keep pulling.
    * decode-rate watch: warn if the 60 s average drops below ``low_fps_warn_hz``.
    """

    def __init__(self, name, port, codec="mjpeg", feed="rs",
                 rotate=None, low_fps_warn_hz=28.0):
        self.name = name
        self.port = int(port)
        self.pipeline = build_pipeline(self.port, codec, feed)
        self.rotate = rotate              # cv2.ROTATE_* applied post-decode, or None
        self.low_fps_warn_hz = low_fps_warn_hz
        # ---- latest-frame store (drop-oldest) ----
        self._cond = threading.Condition()
        self._frame = None                # newest decoded BGR frame or None
        self._recv_t = 0.0                # wall-clock (time.time) of _frame
        self._dirty = False               # new frame since last snapshot()
        # ---- stats ----
        self.connected = False
        self.frames = 0                   # total decoded frames
        self.fps = 0.0                    # rolling rate over the last window
        self.width = 0
        self.height = 0
        # ---- lifecycle ----
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ API
    def start(self):
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name=self.name,
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def snapshot(self):
        """Return ``(frame, recv_t, is_new)``.

        ``is_new`` is True iff a fresh frame arrived since the last snapshot.
        ``frame`` is shared with the decode thread — treat as read-only and do
        not retain it across snapshots.
        """
        with self._cond:
            frame = self._frame
            recv_t = self._recv_t
            is_new = self._dirty
            self._dirty = False
        return frame, recv_t, is_new

    def peek(self):
        """Return ``(frame, recv_t)`` without consuming the new-frame flag."""
        with self._cond:
            return self._frame, self._recv_t

    # ------------------------------------------------------------- internals
    def _publish(self, frame, recv_t):
        with self._cond:
            self._frame = frame
            self._recv_t = recv_t
            self._dirty = True

    def _loop(self):
        while not self._stop.is_set():
            cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                log.warning("[%s] Could not open UDP port %d — is the sender "
                            "running, or is another process holding the port? "
                            "Retrying in 2s...", self.name, self.port)
                cap.release()
                self._stop.wait(2.0)
                continue
            self.connected = True
            log.info("[%s] Connected: UDP port %d (codec %s)",
                     self.name, self.port,
                     "mjpeg" if "JPEG" in self.pipeline else "h264")

            window_n = 0                 # decoded frames in the current 60 s window
            window_t0 = time.monotonic()
            report_n = 0                 # decoded frames since the last fps report
            report_t = window_t0
            warn_n = 0
            warn_t = window_t0

            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok or frame is None or frame.size == 0:
                    # Receiver behaviour: empty pull -> 5 ms nap, keep going.
                    self._stop.wait(0.005)
                    continue

                if self.rotate is not None:
                    frame = cv2.rotate(frame, self.rotate)
                self._publish(frame, time.time())
                self.frames += 1
                h, w = frame.shape[:2]
                self.width, self.height = w, h

                now = time.monotonic()
                window_n += 1
                if now - report_t >= 5.0:
                    self.fps = (window_n - report_n) / (now - report_t)
                    report_n = window_n
                    report_t = now
                    log.info("[%s] fps %.1f (%dx%d, %d total)",
                             self.name, self.fps, self.width, self.height,
                             self.frames)
                if now - warn_t >= 60.0:
                    wfps = (window_n - warn_n) / (now - warn_t)
                    if wfps < self.low_fps_warn_hz:
                        log.warning("[%s] Frame drop detected! Average FPS: %.1f",
                                    self.name, wfps)
                    warn_n = window_n
                    warn_t = now
            cap.release()
        log.info("[%s] Decode thread stopped", self.name)
