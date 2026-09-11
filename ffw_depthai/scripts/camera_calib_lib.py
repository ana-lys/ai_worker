#!/usr/bin/env python3
"""Shared helpers for camera-mount calibration.

Common CSV schema, pose<->matrix conversions, and the hand-eye-style
least-squares correction solver, factored out so the passive recorder, the
active sweep, and the standalone offline solver all agree on the exact same
data model:

    record_camera_calibration_check.py       (passive recorder)
    sweep_and_solve_camera_calibration.py    (active head sweep + solver)
    solve_camera_mount_correction.py         (standalone offline solver, CLI)

Model recap: /head_camera_tf reports A_i = T_baselink_camera using the
CURRENT (possibly wrong) head_camera_frame mount offset. B_i = T_camera_board
is the AprilTag PnP result and does not depend on the URDF at all. If the
mount offset had an unknown fixed error, applying an unknown correction D in
the camera's own local frame should make

    X_i(D) = A_i . D . B_i

constant across every sample i regardless of head position. We solve for the
6-DOF D (translation + axis-angle rotation) that minimizes the spread of
X_i's position over all samples, via nonlinear least squares, with light
regularization toward D=identity so an under-constrained fit (too little
independent head motion) is visible as "the answer collapses once
regularized" rather than silently returned as if it were trustworthy.

D is meant to be applied ON TOP OF the current head_camera_frame mount
transform (new_mount = current_mount . D) -- this module never reads the
URDF or the static_transform_publisher directly.
"""

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

CSV_HEADER = [
    't', 'head_joint1', 'head_joint2',
    'cam_x', 'cam_y', 'cam_z', 'cam_qw', 'cam_qx', 'cam_qy', 'cam_qz',
    'cam2brd_x', 'cam2brd_y', 'cam2brd_z',
    'cam2brd_qw', 'cam2brd_qx', 'cam2brd_qy', 'cam2brd_qz',
    'brd_base_x', 'brd_base_y', 'brd_base_z',
    'brd_base_qw', 'brd_base_qx', 'brd_base_qy', 'brd_base_qz',
]


def pose_to_matrix(x, y, z, qw, qx, qy, qz):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def params_to_matrix(p):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(p[3:6]).as_matrix()
    T[:3, 3] = p[0:3]
    return T


def row_to_matrices(row):
    """row: a dict keyed like CSV_HEADER (e.g. from csv.DictReader) -> (A, B)."""
    A = pose_to_matrix(row['cam_x'], row['cam_y'], row['cam_z'],
                        row['cam_qw'], row['cam_qx'], row['cam_qy'], row['cam_qz'])
    B = pose_to_matrix(row['cam2brd_x'], row['cam2brd_y'], row['cam2brd_z'],
                        row['cam2brd_qw'], row['cam2brd_qx'], row['cam2brd_qy'], row['cam2brd_qz'])
    return A, B


def build_row(t, h1, h2, cam_tf, board_camera_pose, board_base_pose):
    """Build one CSV_HEADER-ordered row.

    cam_tf: geometry_msgs/TransformStamped (/head_camera_tf), T_baselink_camera
    board_camera_pose: geometry_msgs/PoseStamped (/oakd/marker_board_pose_camera_frame),
        T_camera_board
    board_base_pose: geometry_msgs/PoseStamped or None (/oakd/marker_board_pose),
        T_baselink_board -- composed pose, included for convenience/debugging
        only; the solver only uses cam_tf and board_camera_pose. NaN-filled
        if not yet available.
    """
    ct = cam_tf.transform
    bc = board_camera_pose.pose
    nan6 = (float('nan'),) * 7
    if board_base_pose is not None:
        bb = board_base_pose.pose
        bb_vals = (bb.position.x, bb.position.y, bb.position.z,
                   bb.orientation.w, bb.orientation.x, bb.orientation.y, bb.orientation.z)
    else:
        bb_vals = nan6
    return [
        round(t, 4), h1, h2,
        ct.translation.x, ct.translation.y, ct.translation.z,
        ct.rotation.w, ct.rotation.x, ct.rotation.y, ct.rotation.z,
        bc.position.x, bc.position.y, bc.position.z,
        bc.orientation.w, bc.orientation.x, bc.orientation.y, bc.orientation.z,
        *bb_vals,
    ]


def solve_mount_correction(A, B, reg_weight=50.0, fix_translation=None):
    """A, B: lists of 4x4 matrices (T_baselink_camera, T_camera_board), same
    length, matching samples. Returns a dict with the solved correction and
    before/after diagnostics -- see module docstring.

    fix_translation: if given (e.g. (0.0, 0.0, 0.0)), translation is held
    fixed at this value and only the 3 rotation DOF are solved for. Use this
    when there's independent reason to believe the translation offset is
    negligible (e.g. a camera swapped in the same physical mount bracket) --
    it halves the unknowns and can turn an under-constrained 6-DOF fit into
    a well-conditioned 3-DOF one on the same data.
    """
    n_free = 3 if fix_translation is not None else 6
    fixed_t = np.array(fix_translation, dtype=float) if fix_translation is not None else None

    def full_params(p):
        if fixed_t is None:
            return p
        return np.concatenate([fixed_t, p])

    def positions_for(p):
        D = params_to_matrix(full_params(p))
        return np.array([(A[i] @ D @ B[i])[:3, 3] for i in range(len(A))])

    def residuals(p):
        positions = positions_for(p)
        mean_pos = positions.mean(axis=0)
        pos_res = (positions - mean_pos).flatten()
        reg_res = reg_weight * p  # translation (m, if free) + rotvec (rad) terms
        return np.concatenate([pos_res, reg_res])

    def range_and_mean(p):
        positions = positions_for(p)
        return positions.max(axis=0) - positions.min(axis=0), positions.mean(axis=0)

    p0 = np.zeros(n_free)
    before_range, before_mean = range_and_mean(p0)

    result = least_squares(residuals, p0, method='lm', max_nfev=5000)
    after_range, after_mean = range_and_mean(result.x)

    full_p = full_params(result.x)
    dx, dy, dz = full_p[0:3]
    rotvec = full_p[3:6]
    angle_deg = float(np.degrees(np.linalg.norm(rotvec)))
    rpy_deg = np.degrees(Rotation.from_rotvec(rotvec).as_euler('xyz'))
    rmse_cm = float(np.sqrt(np.mean(result.fun ** 2)) * 100)

    return dict(
        success=result.success, message=result.message, p=full_p,
        D=params_to_matrix(full_p),
        before_range=before_range, before_mean=before_mean,
        after_range=after_range, after_mean=after_mean,
        translation=(float(dx), float(dy), float(dz)),
        angle_deg=angle_deg, rpy_deg=rpy_deg, rmse_cm=rmse_cm,
        fixed_translation=fix_translation is not None,
    )


def format_solution_report(sol):
    br, bm = sol['before_range'], sol['before_mean']
    ar, am = sol['after_range'], sol['after_mean']
    lines = [
        f"BEFORE (identity correction): mean=({bm[0]:.4f}, {bm[1]:.4f}, {bm[2]:.4f}) m  "
        f"range=({br[0]*100:.1f}, {br[1]*100:.1f}, {br[2]*100:.1f}) cm",
        f"AFTER  (solved correction):   mean=({am[0]:.4f}, {am[1]:.4f}, {am[2]:.4f}) m  "
        f"range=({ar[0]*100:.1f}, {ar[1]*100:.1f}, {ar[2]*100:.1f}) cm",
    ]
    if not sol['success']:
        lines.append(f"WARNING: optimizer did not converge cleanly: {sol['message']}")
    if sol.get('fixed_translation'):
        lines.append('(translation held fixed -- solved for rotation only)')
    dx, dy, dz = sol['translation']
    rpy = sol['rpy_deg']
    lines += [
        '',
        'Solved correction D (apply ON TOP of the current head_camera_frame '
        'mount transform: new_mount = current_mount . D):',
        f"  translation (m):   x={dx:.5f}  y={dy:.5f}  z={dz:.5f}",
        f"  rotation (axis-angle, deg): {sol['angle_deg']:.3f}",
        f"  rotation (extrinsic rpy, deg): roll={rpy[0]:.3f} pitch={rpy[1]:.3f} yaw={rpy[2]:.3f}",
        f"  residual RMS: {sol['rmse_cm']:.2f} cm",
    ]
    return '\n'.join(lines)
