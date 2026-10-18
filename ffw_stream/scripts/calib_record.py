#!/usr/bin/env python3
"""Record calibration images from ONE camera stream: a single live view, and
every decoded frame saved losslessly to ~/mrcalib/data/<camera>/NNNNNN.png.

  calib_record.py right        # right D405 RGB  (UDP 9003)
  calib_record.py left         # left D405       (UDP 9001)
  calib_record.py head         # head: D435 on 9100 or OAK-D 720p on 9110, whichever is live

Re-running appends: numbering continues after the highest index already in the
folder. q / Esc in the window or Ctrl+C quits.

Frames are saved as the sensor sends them (NOT rotated 90 deg like the
dashboard), so they match the factory intrinsics' image orientation. The codec
(H264 or MJPEG) is detected from the RTP payload type. The UDP port can only be
read by one process: stop the dashboard receiver first."""
import argparse, glob, os, queue, re, signal, socket, sys, threading, time
import cv2

PORTS = {'left': [9001], 'right': [9003], 'head': [9100, 9110]}
OUT_ROOT = os.path.expanduser('~/mrcalib/data')


def probe(ports, timeout):
    """First port that receives an RTP packet, and its payload type."""
    socks = []
    for p in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', p)); s.setblocking(False)
        socks.append((p, s))
    try:
        end = time.time() + timeout
        while time.time() < end:
            for p, s in socks:
                try:
                    data = s.recv(65536)
                except BlockingIOError:
                    continue
                if len(data) >= 12:
                    return p, data[1] & 0x7f
            time.sleep(0.005)
        return None, None
    finally:
        for _, s in socks:
            s.close()


def pipeline(port, pt):
    if pt == 26:   # RFC 2435 JPEG
        return (f'udpsrc port={port} buffer-size=2147483647 '
                'caps="application/x-rtp, media=video, encoding-name=JPEG, payload=26" ! '
                'rtpjpegdepay ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! '
                'appsink sync=false max-buffers=30')
    return (f'udpsrc port={port} buffer-size=2147483647 '
            'caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264" ! '
            'rtpjitterbuffer latency=20 ! rtph264depay ! decodebin ! videoconvert ! '
            'video/x-raw,format=BGR ! appsink sync=false max-buffers=30')


def next_index(folder):
    idx = [int(m.group(1)) for f in glob.glob(os.path.join(folder, '*.png'))
           if (m := re.fullmatch(r'(\d+)\.png', os.path.basename(f)))]
    return max(idx) + 1 if idx else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('camera', choices=sorted(PORTS))
    ap.add_argument('--port', type=int, help='override the UDP port')
    ap.add_argument('--out', default=OUT_ROOT, help='root folder (default ~/mrcalib/data)')
    ap.add_argument('--probe-timeout', type=float, default=5.0)
    ap.add_argument('--view', default='1920x1080', help='live view fits inside WxH (display only; saved frames stay native)')
    a = ap.parse_args()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    port, pt = probe([a.port] if a.port else PORTS[a.camera], a.probe_timeout)
    if port is None:
        sys.exit(f'no packets on UDP {a.port or PORTS[a.camera]} within {a.probe_timeout:.0f} s '
                 '-- is the robot streaming (and the dashboard receiver stopped)?')
    codec = 'MJPEG' if pt == 26 else 'H264'
    cap = cv2.VideoCapture(pipeline(port, pt), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        sys.exit(f'could not open the {codec} pipeline on UDP {port}')

    folder = os.path.join(a.out, a.camera)
    os.makedirs(folder, exist_ok=True)
    first = idx = next_index(folder)
    print(f'{a.camera}: UDP {port} {codec} -> {folder}/ starting at {first:06d}.png  (q / Ctrl+C quits)')

    # PNG encoding is slow next to 30 fps: a writer thread so no frame is dropped
    q = queue.Queue()

    def writer():
        while True:
            item = q.get()
            if item is None:
                return
            cv2.imwrite(*item)
    wt = threading.Thread(target=writer, daemon=True); wt.start()

    win = f'calib_record {a.camera}'
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    vw, vh = (int(v) for v in a.view.lower().split('x'))
    t0, size, scale = time.time(), None, 1.0
    try:
        while not stop.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                if cv2.waitKey(5) & 0xff in (ord('q'), 27):
                    break
                continue
            if size is None:
                size = frame.shape[1::-1]
                scale = min(vw / size[0], vh / size[1])
                print(f'frame size {size[0]}x{size[1]}, view x{scale:.2f}')
            q.put((os.path.join(folder, f'{idx:06d}.png'), frame))
            idx += 1
            view = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST) if scale != 1.0 else frame.copy()
            n = idx - first
            cv2.putText(view, f'{a.camera} #{idx - 1:06d}  rec {n}  {n / max(time.time() - t0, 1e-3):.1f} fps  q=quit',
                        (12, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow(win, view)
            if cv2.waitKey(1) & 0xff in (ord('q'), 27):
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if q.qsize():
            print(f'writing {q.qsize()} queued frames...')
        q.put(None); wt.join()
        print(f'saved {idx - first} frames ({first:06d}..{idx - 1:06d}) in {folder}')


if __name__ == '__main__':
    main()
