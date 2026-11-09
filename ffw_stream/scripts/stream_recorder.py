#!/usr/bin/env python3
"""Receive the robot's D405 streams, show them, and save every frame natively.

Replaces the unified receiver while it runs (same UDP ports -- stop the
receiver first). Nothing is re-encoded:

  * RGB  (RTP/JPEG, ports base+1 / base+3): the JPEG that rtpjpegdepay
    rebuilds from the RTP packets, saved as NNNNNN.jpg. RTP/JPEG carries
    sizes in 8-px units, so a 480x270 stream is stored as 480x272.
  * Depth (FZD1 lossless Z16, ports base+0 / base+2): the zstd payload exactly
    as sent, saved as NNNNNN.zst (codec in the index). Decode with
    `stream_recorder.py decode <recording>` -> 16-bit PNGs (sensor units,
    depth_scale m/unit, 0 = no depth), or decode_depth() from Python.

Layout:
  <out>/meta.json
  <out>/d405_0_depth/  <out>/d405_0_rgb/  <out>/d405_1_depth/  <out>/d405_1_rgb/
  (+ <out>/head_rgb/ with --head, D435 MJPEG on base+100)
  each folder: frame files + index.json (and index.jsonl, written live so a
  crash loses nothing). One record per frame:
    {"id": 0, "file": "000000.jpg", "recv_ns": ..., "bytes": ...}
  depth also: sender_frame_id, stamp_ns (robot capture, CLOCK_REALTIME),
  width, height, depth_scale, codec.

Usage:
  stream_recorder.py [--out DIR] [--head] [--no-show]
  stream_recorder.py decode DIR        # depth .zst -> depth_png/NNNNNN.png
Quit: q / Esc in the window, or Ctrl+C in the terminal.
"""
import argparse
import ctypes
import ctypes.util
import datetime
import json
import os
import signal
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np

# ---------------------------------------------------------------- depth wire
MAGIC = 0x31445A46  # "FZD1"
HDR = struct.Struct('<IIHHHHB3xfIQ')  # must match DepthChunkHeader (36 B)
assert HDR.size == 36
CODEC_NAMES = {0: 'raw', 1: 'zstd', 2: 'zstd_delta_planes'}

_zstd = ctypes.CDLL(ctypes.util.find_library('zstd') or 'libzstd.so.1')
_zstd.ZSTD_decompress.restype = ctypes.c_size_t
_zstd.ZSTD_decompress.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]
_zstd.ZSTD_isError.restype = ctypes.c_uint
_zstd.ZSTD_isError.argtypes = [ctypes.c_size_t]


def decode_depth(payload: bytes, width: int, height: int, codec: int) -> np.ndarray:
    """Native depth payload -> uint16 (height, width) in sensor units."""
    n = width * height * 2
    if codec == 0:
        return np.frombuffer(payload, np.uint16, width * height).reshape(height, width).copy()
    out = ctypes.create_string_buffer(n)
    r = _zstd.ZSTD_decompress(out, n, payload, len(payload))
    if _zstd.ZSTD_isError(r) or r != n:
        raise ValueError('zstd decompress failed')
    buf = np.frombuffer(out.raw, np.uint8)
    if codec == 1:
        return buf.view(np.uint16).reshape(height, width).copy()
    # codec 2: hi plane, lo plane of zigzag(row-wise left delta)
    hi = buf[:width * height].astype(np.uint16)
    lo = buf[width * height:].astype(np.uint16)
    zz = ((hi << 8) | lo).reshape(height, width)
    d = (zz >> 1) ^ (np.uint16(0) - (zz & 1)).astype(np.uint16)
    return np.cumsum(d, axis=1, dtype=np.uint16)  # wraps mod 2^16 like the C++


# ---------------------------------------------------------------- writers
class StreamDir:
    def __init__(self, root, name):
        self.path = os.path.join(root, name)
        os.makedirs(self.path, exist_ok=True)
        self.name = name
        self.records = []
        self.live = open(os.path.join(self.path, 'index.jsonl'), 'w')
        self.lock = threading.Lock()

    def save(self, data: bytes, ext: str, **fields):
        with self.lock:
            fid = len(self.records)
            fname = f'{fid:06d}.{ext}'
            with open(os.path.join(self.path, fname), 'wb') as f:
                f.write(data)
            rec = {'id': fid, 'file': fname, 'recv_ns': time.time_ns(), 'bytes': len(data), **fields}
            self.records.append(rec)
            self.live.write(json.dumps(rec) + '\n')
            self.live.flush()
            return fid

    def close(self):
        with self.lock:
            self.live.close()
            with open(os.path.join(self.path, 'index.json'), 'w') as f:
                json.dump({'stream': self.name, 'frames': self.records}, f, indent=1)


def port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(('0.0.0.0', port))
        return True
    except OSError:
        return False
    finally:
        s.close()


# ---------------------------------------------------------------- receivers
def depth_thread(port, sdir, latest, key, stop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(0.2)
    cur, chunks, dropped = None, {}, 0
    while not stop.is_set():
        try:
            pkt = sock.recv(2048)
        except socket.timeout:
            continue
        if len(pkt) < HDR.size:
            continue
        magic, fid, idx, count, w, h, codec, scale, psize, stamp = HDR.unpack_from(pkt)
        if magic != MAGIC:
            continue
        if fid != cur:
            if cur is not None and chunks:
                dropped += 1  # previous frame never completed
            cur, chunks = fid, {}
        chunks[idx] = pkt[HDR.size:]
        if len(chunks) == count:
            payload = b''.join(chunks[i] for i in range(count))
            chunks = {}
            if len(payload) != psize:
                dropped += 1
                continue
            n = sdir.save(payload, 'zst' if codec else 'raw', sender_frame_id=fid, stamp_ns=stamp,
                          width=w, height=h, depth_scale=scale, codec=CODEC_NAMES.get(codec, codec),
                          dropped_so_far=dropped)
            latest[key] = ('depth', payload, w, h, codec, scale, n)
    sock.close()


def jpeg_thread(port, sdir, latest, key, stop):
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)
    pipe = Gst.parse_launch(
        f'udpsrc port={port} reuse=false buffer-size=33554432 '
        'caps="application/x-rtp, media=video, encoding-name=JPEG, payload=26, clock-rate=90000" ! '
        'rtpjpegdepay ! appsink name=sink sync=false emit-signals=false max-buffers=256 drop=false')
    sink = pipe.get_by_name('sink')
    pipe.set_state(Gst.State.PLAYING)
    while not stop.is_set():
        sample = sink.emit('try-pull-sample', 200 * Gst.MSECOND)
        if sample is None:
            continue
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            continue
        data = bytes(info.data)
        buf.unmap(info)
        n = sdir.save(data, 'jpg')
        latest[key] = ('jpeg', data, n)
    pipe.set_state(Gst.State.NULL)


# ---------------------------------------------------------------- display
def depth_vis(z16, scale, max_m=1.0):
    m = z16.astype(np.float32) * scale
    v = np.clip(m / max_m, 0, 1)
    img = cv2.applyColorMap((255 * (1 - v)).astype(np.uint8), cv2.COLORMAP_JET)
    img[z16 == 0] = 0
    return img


def render(latest, keys, tile_h=480):
    tiles = []
    for key in keys:
        item = latest.get(key)
        img = None
        if item and item[0] == 'jpeg':
            img = cv2.imdecode(np.frombuffer(item[1], np.uint8), cv2.IMREAD_COLOR)
            n = item[2]
        elif item and item[0] == 'depth':
            _, payload, w, h, codec, scale, n = item
            try:
                img = depth_vis(decode_depth(payload, w, h, codec), scale)
            except ValueError:
                img = None
        if img is None:
            img = np.zeros((480, 270, 3), np.uint8)
            n = 0
        elif key != 'head_rgb':
            img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)  # same view as the receiver
        img = cv2.resize(img, (round(img.shape[1] * tile_h / img.shape[0]), tile_h))
        cv2.putText(img, f'{key} #{n}', (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        tiles.append(img)
    return cv2.hconcat(tiles)


# ---------------------------------------------------------------- main
def record(args):
    out = args.out or os.path.expanduser(
        os.path.join('~/ffw_recordings', datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
    b = args.base_port
    streams = [  # (folder, kind, port)
        ('d405_0_depth', 'depth', b + 0), ('d405_0_rgb', 'jpeg', b + 1),
        ('d405_1_depth', 'depth', b + 2), ('d405_1_rgb', 'jpeg', b + 3),
    ]
    if args.head:
        streams.append(('head_rgb', 'jpeg', b + 100))
    busy = [p for _, _, p in streams if not port_free(p)]
    if busy:
        sys.exit(f'UDP ports {busy} are in use -- stop the unified receiver first '
                 '(pkill -f realsense_udp_receiver)')

    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'meta.json'), 'w') as f:
        json.dump({'start_ns': time.time_ns(), 'start_iso': datetime.datetime.now().isoformat(),
                   'streams': {n: {'kind': k, 'port': p} for n, k, p in streams},
                   'note': 'rgb = native JPEG from RTP (480x272 for a 480x270 stream); '
                           'depth = native FZD1 payload, decode with stream_recorder.py decode'},
                  f, indent=1)

    stop = threading.Event()
    latest, dirs, threads = {}, {}, []
    for name, kind, port in streams:
        dirs[name] = StreamDir(out, name)
        fn = depth_thread if kind == 'depth' else jpeg_thread
        t = threading.Thread(target=fn, args=(port, dirs[name], latest, name, stop), daemon=True)
        t.start()
        threads.append(t)

    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print(f'Recording to {out}  (q/Esc or Ctrl+C to stop)')
    keys = [n for n, _, _ in streams]
    win = 'stream_recorder'
    last_print = time.time()
    try:
        while not stop.is_set():
            if not args.no_show:
                cv2.imshow(win, render(latest, keys))
                k = cv2.waitKey(30) & 0xFF
                if k in (ord('q'), 27):
                    break
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break  # window closed with the X button
            else:
                time.sleep(0.1)
            if time.time() - last_print > 5:
                last_print = time.time()
                print('  ' + '  '.join(f'{n}={len(d.records)}' for n, d in dirs.items()), flush=True)
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2)
        for d in dirs.values():
            d.close()
        if not args.no_show:
            cv2.destroyAllWindows()
        with open(os.path.join(out, 'meta.json')) as f:
            meta = json.load(f)
        meta['stop_ns'] = time.time_ns()
        meta['frames'] = {n: len(d.records) for n, d in dirs.items()}
        with open(os.path.join(out, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        print(f'Saved {meta["frames"]} -> {out}')


def decode(args):
    for sub in sorted(os.listdir(args.dir)):
        idx = os.path.join(args.dir, sub, 'index.json')
        if not (sub.endswith('_depth') and os.path.exists(idx)):
            continue
        with open(idx) as f:
            frames = json.load(f)['frames']
        outdir = os.path.join(args.dir, sub, 'depth_png')
        os.makedirs(outdir, exist_ok=True)
        codes = {v: k for k, v in CODEC_NAMES.items()}
        for r in frames:
            with open(os.path.join(args.dir, sub, r['file']), 'rb') as f:
                z = decode_depth(f.read(), r['width'], r['height'], codes[r['codec']])
            cv2.imwrite(os.path.join(outdir, r['file'].rsplit('.', 1)[0] + '.png'), z)
        print(f'{sub}: {len(frames)} frames -> {outdir} (uint16, x depth_scale = meters)')


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'decode':
        p = argparse.ArgumentParser(prog='stream_recorder.py decode')
        p.add_argument('dir')
        decode(p.parse_args(sys.argv[2:]))
        return
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', help='output folder (default ~/ffw_recordings/<date_time>)')
    p.add_argument('--base-port', type=int, default=9000)
    p.add_argument('--head', action='store_true', help='also record the D435 head MJPEG (base+100)')
    p.add_argument('--no-show', action='store_true', help='record without the window')
    record(p.parse_args())


if __name__ == '__main__':
    main()
