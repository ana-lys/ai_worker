#!/usr/bin/env python3
"""Solves for the head_camera_frame URDF mount-offset correction from a
record_camera_calibration_check.py (or sweep_and_solve_camera_calibration.py)
CSV.

See camera_calib_lib.py for the full model description. In short: this
solves for a 6-DOF correction D such that A_i . D . B_i is constant across
every recorded sample, where A_i = T_baselink_camera (/head_camera_tf) and
B_i = T_camera_board (/oakd/marker_board_pose_camera_frame).

Usage:
    python3 solve_camera_mount_correction.py /path/to/camera_calib_check.csv
"""

import csv
import sys

from camera_calib_lib import format_solution_report, row_to_matrices, solve_mount_correction


def load_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    rows = load_csv(sys.argv[1])
    print(f"Loaded {len(rows)} samples")

    matrices = [row_to_matrices(r) for r in rows]
    A = [m[0] for m in matrices]
    B = [m[1] for m in matrices]

    sol = solve_mount_correction(A, B, reg_weight=50.0)
    print(format_solution_report(sol))


if __name__ == '__main__':
    main()
