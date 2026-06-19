#!/usr/bin/env python3
"""
UDP depth/IR receiver.

Each stream sent by udp_depth_ir_streamer is an MJPEG-in-MPEGTS UDP stream
of raw 8-bit grayscale frames (depth clamped+scaled to [0, max_depth_m], or IR).

This spawns one ffmpeg subprocess per stream to decode it back to raw 8-bit
frames, then shows each stream in its own OpenCV window.

Requires: ffmpeg on PATH, python packages numpy + opencv-python.

Usage example (2 cameras, depth+ir each):
    python3 udp_receiver.py --width 480 --height 270 \
        --stream depth0:5000 --stream ir0:5001 \
        --stream depth1:5002 --stream ir1:5003

Press 'q' in any window to quit.
"""

import argparse
import subprocess
import threading
import sys

import numpy as np
import cv2


def reader_thread(label, port, width, height, stop_event, frame_holder, lock):
    frame_size = width * height
    cmd = [
        "ffmpeg",
        "-hide_banner", "-loglevel", "error",
        "-fflags", "nobuffer",
        "-i", f"udp://0.0.0.0:{port}",
        "-f", "rawvideo",
        "-pix_fmt", "gray",
        "-",
    ]
    print(f"[{label}] starting: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=frame_size * 2)

    try:
        while not stop_event.is_set():
            raw = proc.stdout.read(frame_size)
            if len(raw) != frame_size:
                # stream ended, ffmpeg died, or partial read on shutdown
                break
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
            with lock:
                frame_holder[label] = frame.copy()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
        print(f"[{label}] stopped")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=480)
    parser.add_argument("--height", type=int, default=270)
    parser.add_argument("--stream", action="append", required=True,
                         help="label:port, e.g. depth0:5000  (repeatable)")
    args = parser.parse_args()

    streams = []
    for s in args.stream:
        label, port = s.split(":")
        streams.append((label, int(port)))

    stop_event = threading.Event()
    frame_holder = {}
    lock = threading.Lock()

    threads = []
    for label, port in streams:
        t = threading.Thread(
            target=reader_thread,
            args=(label, port, args.width, args.height, stop_event, frame_holder, lock),
            daemon=True,
        )
        t.start()
        threads.append(t)

    try:
        while True:
            with lock:
                items = list(frame_holder.items())
            for label, frame in items:
                cv2.imshow(label, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=2)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()