#!/usr/bin/env python3
"""RealSense depth health check (on-chip calibration, READ-ONLY) + flatness test.

For every connected D4xx (or the ones given with --serial):
  1. Runs Intel's on-chip self-calibration in check mode and prints the
     health score. The new calibration table it computes is NOT written --
     nothing on the camera changes.
        |health| < 0.25   good, leave it
        0.25 .. 0.75      can be improved (run the real on-chip calibration)
        > 0.75            needs calibration
  2. Optional flatness test (--plane, default on): streams ~1 s of depth,
     fits a plane to the central ROI and reports fill rate, distance and
     plane-fit RMS (the same "depth noise" on-chip calibration improves).

Target: a FLAT surface filling most of the view, camera roughly
perpendicular to it.
  * D405 (no IR projector): the surface must be TEXTURED -- e.g. a printed
    random-dot / newspaper-like pattern on stiff board. A plain white wall
    gives no stereo matches. Distance ~20-30 cm.
  * D435 (has projector): a plain flat wall at ~0.6-1.5 m is fine.
Hold still (camera and target) during the ~10-30 s check, even light, no
shiny surfaces. Auto-exposure is switched ON for the check (the streamer's
fixed --color-exposure also pins the D405 depth imager); --keep-exposure
skips that. The streamer re-applies its own exposure when it restarts.

The cameras must be FREE: stop realsense_udp_streamer first (bringup with
launch_cameras:=false, or kill the streamer process).

Usage (inside the ai_worker container):
  d405_health_check.py                    # all cameras
  d405_health_check.py --serial 1234567   # one camera
  d405_health_check.py --speed 3 --no-plane
Results also go to --out (default /workspace/realsense_health/<date>.json).
"""
import argparse
import datetime
import json
import os
import sys
import time

import numpy as np
import pyrealsense2 as rs

SPEEDS = {0: 'very fast', 1: 'fast', 2: 'medium', 3: 'slow', 4: 'white wall'}


def verdict(h):
    a = abs(h)
    if a < 0.25:
        return 'GOOD - no calibration needed'
    if a < 0.75:
        return 'CAN BE IMPROVED - on-chip calibration recommended'
    return 'NEEDS CALIBRATION'


def depth_profiles(dev):
    sensor = dev.first_depth_sensor()
    out = []
    for p in sensor.get_stream_profiles():
        if p.stream_type() == rs.stream.depth and p.format() == rs.format.z16:
            v = p.as_video_stream_profile()
            out.append((v.width(), v.height(), p.fps()))
    return sorted(set(out))


def pick(profiles, wanted):
    for w in wanted:
        if w in profiles:
            return w
    return None


AUTO_EXPOSURE = True  # set from --keep-exposure in main()


def start(serial, prof):
    """Start depth and return (pipe, profile). ALWAYS talk to the camera via
    profile.get_device() afterwards: a device handle from another rs.context()
    cannot reach a camera this pipeline holds ('failed to set power state')."""
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth, prof[0], prof[1], rs.format.z16, prof[2])
    profile = pipe.start(cfg)
    if AUTO_EXPOSURE:
        # The streamer's --color-exposure pins the SHARED imager (D405 RGB
        # comes from the depth imager) with auto-exposure off, and that stays
        # on the camera after the streamer exits. Calibrate with auto-exposure.
        try:
            ds = profile.get_device().first_depth_sensor()
            if ds.supports(rs.option.enable_auto_exposure):
                ds.set_option(rs.option.enable_auto_exposure, 1)
        except Exception as e:
            print(f'  could not enable auto-exposure: {e}')
    for _ in range(30):  # let auto-exposure settle
        pipe.wait_for_frames(5000)
    return pipe, profile


def health_check(dev, serial, speed, timeout_s):
    prof = pick(depth_profiles(dev), [(256, 144, 90), (848, 480, 30), (1280, 720, 30), (640, 480, 30)])
    if prof is None:
        raise RuntimeError('no usable depth profile')
    pipe, profile = start(serial, prof)
    try:
        ac = profile.get_device().as_auto_calibrated_device()
        cfg = json.dumps({'calib type': 0, 'speed': speed, 'scan parameter': 0,
                          'white_wall_mode': 1 if speed == 4 else 0})
        last = [-10]

        def progress(p):
            if p - last[0] >= 10:
                last[0] = p
                print(f'    progress {p:5.1f} %', flush=True)

        t0 = time.time()
        _table, health = ac.run_on_chip_calibration(cfg, progress, int(timeout_s * 1000))
        return {'profile': prof, 'health': float(health[0]), 'health_extra': float(health[1]),
                'seconds': round(time.time() - t0, 1)}
    finally:
        pipe.stop()  # table discarded: nothing written to the camera


def plane_test(dev, serial, n_frames=30, roi=0.5):
    prof = pick(depth_profiles(dev), [(848, 480, 30), (640, 480, 30), (1280, 720, 30), (480, 270, 15)])
    pipe, profile = start(serial, prof)
    try:
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        w, h = intr.width, intr.height
        x0, x1 = int(w * (1 - roi) / 2), int(w * (1 + roi) / 2)
        y0, y1 = int(h * (1 - roi) / 2), int(h * (1 + roi) / 2)
        us, vs = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
        rms, fills, dists = [], [], []
        for _ in range(n_frames):
            z = np.asanyarray(pipe.wait_for_frames(5000).get_depth_frame().get_data())[y0:y1, x0:x1]
            m = z > 0
            fills.append(m.mean())
            if m.sum() < 500:
                continue
            Z = z[m].astype(np.float64) * scale
            X = (us[m] - intr.ppx) / intr.fx * Z
            Y = (vs[m] - intr.ppy) / intr.fy * Z
            A = np.c_[X, Y, np.ones_like(X)]
            coef, *_ = np.linalg.lstsq(A, Z, rcond=None)  # Z = aX + bY + c
            res = Z - A @ coef
            n = np.linalg.norm([coef[0], coef[1], -1.0])
            rms.append(np.sqrt(np.mean(res ** 2)) / n)  # perpendicular residual
            dists.append(np.median(Z))
        if not rms:
            return {'profile': prof, 'fill_pct': round(100 * float(np.mean(fills)), 1),
                    'error': 'too few valid pixels -- target textured / in range?'}
        d = float(np.median(dists))
        r = float(np.median(rms))
        return {'profile': prof, 'roi_pct': int(roi * 100), 'fill_pct': round(100 * float(np.mean(fills)), 1),
                'distance_m': round(d, 4), 'plane_rms_mm': round(r * 1000, 3),
                'plane_rms_pct_of_distance': round(100 * r / d, 3)}
    finally:
        pipe.stop()


def aim(dev, serial, target_m, countdown_s=3.0):
    """Live text readout to line the target up (no display needed over ssh).
    Continues by itself once the target has stayed OK for countdown_s
    seconds (the countdown restarts whenever it goes out of tolerance);
    Enter continues immediately."""
    import select
    prof = pick(depth_profiles(dev), [(848, 480, 30), (640, 480, 30), (1280, 720, 30), (480, 270, 15)])
    pipe, profile = start(serial, prof)
    try:
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        w, h = intr.width, intr.height
        x0, x1, y0, y1 = w // 4, 3 * w // 4, h // 4, 3 * h // 4
        us, vs = np.meshgrid(np.arange(x0, x1, 2), np.arange(y0, y1, 2))
        ok_since = None
        print(f'  AIM: line up the target; it starts by itself after {countdown_s:.0f} s OK '
              '(Enter = start now)  '
              f'(want: fill > 80 %, distance ~{target_m*100:.0f} cm, tilt < 3 deg)')
        while True:
            z_full = np.asanyarray(pipe.wait_for_frames(5000).get_depth_frame().get_data())
            fill = 100.0 * (z_full > 0).mean()
            z = z_full[y0:y1:2, x0:x1:2]
            m = z > 0
            msg = f'fill {fill:5.1f} %'
            hints = []
            if m.sum() > 200:
                Z = z[m].astype(np.float64) * scale
                X = (us[m] - intr.ppx) / intr.fx * Z
                Y = (vs[m] - intr.ppy) / intr.fy * Z
                A = np.c_[X, Y, np.ones_like(X)]
                (a, b, c), *_ = np.linalg.lstsq(A, Z, rcond=None)
                res = Z - A @ np.array([a, b, c])
                d = float(np.median(Z))
                tx, ty = np.degrees(np.arctan(a)), np.degrees(np.arctan(b))
                rms = np.sqrt(np.mean(res ** 2)) * 1000
                msg += f' | dist {d*100:5.1f} cm | tilt L/R {tx:+5.1f} U/D {ty:+5.1f} deg | rms {rms:5.2f} mm'
                if d > target_m * 1.25: hints.append('closer')
                if d < target_m * 0.75: hints.append('farther')
                if abs(tx) > 3 or abs(ty) > 3: hints.append('face-on')
            else:
                hints.append('no depth: target in view? textured? light?')
            if fill < 80: hints.append('fill the view / more texture or light')
            now = time.time()
            if hints:
                ok_since = None
                msg += '  -> ' + ', '.join(hints)
            else:
                ok_since = ok_since or now
                left = countdown_s - (now - ok_since)
                if left <= 0:
                    print('\r  ' + (msg + '  -> OK, starting').ljust(150), flush=True)
                    return
                msg += f'  -> OK, hold still... starting in {left:3.1f} s'
            print('\r  ' + msg[:150].ljust(150), end='', flush=True)
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                print()
                return
    finally:
        pipe.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--serial', action='append', help='camera serial (repeatable); default all')
    ap.add_argument('--speed', type=int, default=2, choices=SPEEDS, help='on-chip speed (default 2 = medium)')
    ap.add_argument('--timeout', type=float, default=60, help='seconds per camera')
    ap.add_argument('--no-plane', action='store_true', help='skip the plane-fit flatness test')
    ap.add_argument('--out', default=None, help='JSON result file')
    ap.add_argument('--no-aim', action='store_true', help='skip the live aiming readout')
    ap.add_argument('--countdown', type=float, default=3.0,
                    help='seconds the aim must stay OK before the check starts by itself (default 3)')
    ap.add_argument('--target-m', type=float, default=None,
                    help='aiming distance hint (default 0.20 m D405, 1.0 m others)')
    ap.add_argument('--keep-exposure', action='store_true',
                    help='keep the current (e.g. streamer manual) exposure instead of auto-exposure')
    args = ap.parse_args()
    global AUTO_EXPOSURE
    AUTO_EXPOSURE = not args.keep_exposure

    ctx = rs.context()
    devs = {d.get_info(rs.camera_info.serial_number): d for d in ctx.query_devices()}
    if not devs:
        sys.exit('No RealSense found.')
    serials = args.serial or sorted(devs)
    results = {'time': datetime.datetime.now().isoformat(), 'speed': SPEEDS[args.speed], 'cameras': {}}

    for s in serials:
        if s not in devs:
            print(f'[{s}] not connected (have {sorted(devs)})')
            continue
        dev = devs[s]
        name = dev.get_info(rs.camera_info.name)
        fw = dev.get_info(rs.camera_info.firmware_version)
        print(f'\n=== {name}  serial {s}  FW {fw} ===')
        r = {'name': name, 'firmware': fw}
        if not dev.is_auto_calibrated_device():
            print('  on-chip calibration not supported by this device/firmware')
            r['health'] = None
        else:
            if not args.no_aim:
                try:
                    aim(dev, s, args.target_m or (0.20 if 'D405' in name else 1.0), args.countdown)
                except Exception as e:
                    print(f'  aim view failed ({e}), continuing')
            print(f'  health check ({SPEEDS[args.speed]}) -- keep camera and target still...')
            try:
                r.update(health_check(dev, s, args.speed, args.timeout))
                print(f"  HEALTH = {r['health']:+.3f}  ->  {verdict(r['health'])}   ({r['seconds']} s)")
                r['verdict'] = verdict(r['health'])
            except Exception as e:  # busy camera, bad target, timeout...
                msg = str(e)
                if 'busy' in msg.lower() or 'power state' in msg.lower():
                    msg += '  <- camera in use: stop realsense_udp_streamer first'
                print(f'  health check FAILED: {msg}')
                r['health_error'] = msg
        if not args.no_plane and 'health_error' not in r:
            try:
                p = plane_test(dev, s)
                r['plane'] = p
                if 'error' in p:
                    print(f"  flatness: {p['error']} (fill {p['fill_pct']} %)")
                else:
                    print(f"  flatness: distance {p['distance_m']*1000:.1f} mm, fill {p['fill_pct']} %, "
                          f"plane RMS {p['plane_rms_mm']:.2f} mm ({p['plane_rms_pct_of_distance']:.2f} % of distance)")
            except Exception as e:
                print(f'  flatness test FAILED: {e}')
        results['cameras'][s] = r

    out = args.out or os.path.join('/workspace/realsense_health',
                                   datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
    try:
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as f:
            json.dump(results, f, indent=1)
        print(f'\nSaved {out}')
    except OSError as e:
        print(f'\n(could not save {out}: {e})')
    print('Nothing was written to the cameras.')


if __name__ == '__main__':
    main()
