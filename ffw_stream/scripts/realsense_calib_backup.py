#!/usr/bin/env python3
"""Save / restore RealSense depth calibration tables (D405, D435, ...).

  realsense_calib_backup.py backup [--serial SN]     # save all (or one) camera
  realsense_calib_backup.py show FILE                # what a backup contains
  realsense_calib_backup.py restore FILE [--serial SN]   # write a backup back
  realsense_calib_backup.py factory --serial SN      # Intel factory calibration

backup is read-only. restore / factory write the camera's flash and ask for
confirmation first (--yes skips it). A backup is taken automatically before
every restore / factory reset.

Backups: /workspace/realsense_calib/<serial>_<date>.json (persistent volume),
holding the raw table plus readable intrinsics / baseline for reference.

The cameras must be free: stop realsense_udp_streamer first.
"""
import argparse
import datetime
import json
import os
import sys

import pyrealsense2 as rs

OUT_DIR = '/workspace/realsense_calib'


def devices():
    return {d.get_info(rs.camera_info.serial_number): d for d in rs.context().query_devices()}


def readable(dev):
    """Intrinsics of the default depth profile + stereo baseline (reference only)."""
    info = {}
    try:
        sensor = dev.first_depth_sensor()
        prof = next(p for p in sensor.get_stream_profiles()
                    if p.stream_type() == rs.stream.depth and p.is_default())
        i = prof.as_video_stream_profile().get_intrinsics()
        info['depth_profile'] = [i.width, i.height, prof.fps()]
        info['fx'], info['fy'], info['ppx'], info['ppy'] = i.fx, i.fy, i.ppx, i.ppy
        info['depth_scale'] = sensor.get_depth_scale()
        ir = [p for p in sensor.get_stream_profiles() if p.stream_type() == rs.stream.infrared]
        left = next((p for p in ir if p.stream_index() == 1), None)
        right = next((p for p in ir if p.stream_index() == 2), None)
        if left and right:
            info['baseline_mm'] = abs(left.get_extrinsics_to(right).translation[0]) * 1000
    except Exception as e:
        info['note'] = f'readable info unavailable: {e}'
    return info


def backup_one(sn, dev, tag='backup'):
    if not dev.is_auto_calibrated_device():
        print(f'[{sn}] does not expose a calibration table, skipped')
        return None
    table = list(dev.as_auto_calibrated_device().get_calibration_table())
    rec = {'serial': sn, 'name': dev.get_info(rs.camera_info.name),
           'firmware': dev.get_info(rs.camera_info.firmware_version),
           'time': datetime.datetime.now().isoformat(), 'tag': tag,
           'readable': readable(dev), 'table_len': len(table), 'table': table}
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"{sn}_{datetime.datetime.now():%Y%m%d_%H%M%S}_{tag}.json")
    with open(path, 'w') as f:
        json.dump(rec, f, indent=1)
    r = rec['readable']
    print(f"[{sn}] {rec['name']} FW {rec['firmware']}: saved {len(table)} B -> {path}")
    if 'fx' in r:
        print(f"      depth {r['depth_profile']} fx={r['fx']:.2f} fy={r['fy']:.2f} "
              f"ppx={r['ppx']:.2f} ppy={r['ppy']:.2f} baseline={r.get('baseline_mm', float('nan')):.3f} mm")
    return path


def confirm(msg, yes):
    if yes:
        return True
    return input(f'{msg} [y/N] ').strip().lower() in ('y', 'yes')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('backup'); b.add_argument('--serial')
    s = sub.add_parser('show'); s.add_argument('file')
    r = sub.add_parser('restore'); r.add_argument('file'); r.add_argument('--serial'); r.add_argument('--yes', action='store_true')
    f = sub.add_parser('factory'); f.add_argument('--serial', required=True); f.add_argument('--yes', action='store_true')
    a = ap.parse_args()

    if a.cmd == 'show':
        rec = json.load(open(a.file))
        print(json.dumps({k: v for k, v in rec.items() if k != 'table'}, indent=1))
        return

    devs = devices()
    if not devs:
        sys.exit('No RealSense found (or all busy -- stop realsense_udp_streamer).')

    if a.cmd == 'backup':
        for sn in ([a.serial] if a.serial else sorted(devs)):
            if sn not in devs:
                print(f'[{sn}] not connected (have {sorted(devs)})')
                continue
            backup_one(sn, devs[sn])
        return

    if a.cmd == 'restore':
        rec = json.load(open(a.file))
        sn = a.serial or rec['serial']
        if sn not in devs:
            sys.exit(f'[{sn}] not connected (have {sorted(devs)})')
        dev = devs[sn]
        if sn != rec['serial']:
            print(f'WARNING: backup is from camera {rec["serial"]}, writing to {sn} -- calibration is per camera!')
        if not confirm(f'Write backup from {rec["time"]} to camera {sn} flash?', a.yes):
            sys.exit('Cancelled, nothing written.')
        backup_one(sn, dev, 'before_restore')
        ac = dev.as_auto_calibrated_device()
        ac.set_calibration_table(rec['table'])
        ac.write_calibration()
        print(f'[{sn}] restored. Unplug/replug or hardware_reset for all streams to pick it up.')
        return

    if a.cmd == 'factory':
        if a.serial not in devs:
            sys.exit(f'[{a.serial}] not connected (have {sorted(devs)})')
        dev = devs[a.serial]
        if not confirm(f'Reset camera {a.serial} to Intel FACTORY calibration?', a.yes):
            sys.exit('Cancelled, nothing written.')
        backup_one(a.serial, dev, 'before_factory')
        dev.as_auto_calibrated_device().reset_to_factory_calibration()
        print(f'[{a.serial}] factory calibration restored.')


if __name__ == '__main__':
    main()
