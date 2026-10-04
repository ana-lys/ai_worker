"""Frame obs block for the real robot, from raw measurements (no ROS; wire.py has the layout).

  joints      25 x [pos, vel, effort] interleaved per joint, wire.JOINT_STATE_NAMES order
              (arm effort in A, the rest in driver units -- the gateway's joint block)
  ee          per EE (right, left): pose (x, y, z, roll, pitch, yaw; base_link,
              R = Rx Ry Rz, the gateway's Obs.ee) then velocity (vx, vy, vz, wx, wy, wz;
              base_link, angular velocity as a vector, not rpy rates) -- finite difference
              of the pose between consecutive frames
  limit       per EE: (lo, hi) margins on x, y, z, roll, pitch, yaw to the arm's GLOBAL limit
              profile box (`peg_hole_safe_real_<l|r>` in limit_profiles.txt), the IK EE pose
              expressed in the profile's TF frame (`peg_hole_frame_<l|r>`), interleaved
              x_lo, x_hi, y_lo, ..., yaw_hi like the gateway's diff block; lo = v - min,
              hi = max - v, both > 0 inside, < 0 past that bound. wire.NO_LIMIT_DIFF on every
              axis of an arm whose frame or profile is missing.

The robot gateway's own diff0/diff1 only track profiles in `marker_frame`, so with the
peg-hole profiles they read NO_LIMIT_DIFF; this module recomputes them per arm frame.
"""
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import wire

PROFILE_FILE = Path(__file__).resolve().parents[2] / "ffw_collision_checker" / "config" / "limit_profiles.txt"
PROFILE = "peg_hole_safe_real"               # + "_l" / "_r"


def rpy(R):
    """R = Rx(roll) Ry(pitch) Rz(yaw) -> (roll, pitch, yaw): the Obs / joy_hand convention."""
    pitch = np.arcsin(np.clip(R[0, 2], -1.0, 1.0))
    return np.array([np.arctan2(-R[1, 2], R[2, 2]), pitch, np.arctan2(-R[0, 1], R[0, 0])])


def pose6(T):
    return np.r_[T[:3, 3], rpy(T[:3, :3])]


def joints_interleaved(pos, vel, eff):
    """(25,) x 3 -> (75,): [pos0, vel0, eff0, pos1, ...]."""
    return np.stack([pos, vel, eff], axis=1).reshape(-1)


def velocity(T_prev, T_now, dt):
    """(vx, vy, vz, wx, wy, wz) in base_link between two poses dt apart; zeros if dt <= 0."""
    if T_prev is None or dt <= 0.0:
        return np.zeros(6)
    w = Rot.from_matrix(T_now[:3, :3] @ T_prev[:3, :3].T).as_rotvec()
    return np.r_[(T_now[:3, 3] - T_prev[:3, 3]) / dt, w / dt]


def load_profiles(name=PROFILE, path=PROFILE_FILE):
    """{'l': (frame, lo(6), hi(6)), 'r': ...} from limit_profiles.txt sections [name_l] / [name_r]."""
    text = Path(path).read_text()
    out = {}
    for arm in ("l", "r"):
        m = re.search(rf"^\[{re.escape(name)}_{arm}\]\n((?:(?!\[).*\n?)*)", text, re.M)
        if not m:
            continue
        body = m.group(1)
        frame = re.search(r"^frame\s+(\S+)", body, re.M)
        vals = re.search(rf"^set\s+{arm}\s+(.+)$", body, re.M)
        if frame and vals:
            v = np.array([float(x) for x in vals.group(1).split()])
            out[arm] = (frame.group(1), v[0::2], v[1::2])
    return out


def limit_margins(T_site, T_frame, lo, hi):
    """(12,) x_lo, x_hi, ..., yaw_hi margins of an IK EE pose (base_link) to a box in T_frame."""
    if T_site is None or T_frame is None:
        return np.full(12, wire.NO_LIMIT_DIFF)
    v = pose6(np.linalg.inv(T_frame) @ T_site)
    return np.stack([v - lo, hi - v], axis=1).reshape(-1)


def build_obs(pos, vel, eff, ee, ee_vel, limits):
    """ee, ee_vel, limits: dicts keyed 'right' / 'left' (pose6, (6,), (12,)) -> (OBS_N,) vector."""
    parts = [joints_interleaved(pos, vel, eff)]
    parts += [np.r_[ee[a], ee_vel[a]] for a in ("right", "left")]
    parts += [limits[a] for a in ("right", "left")]
    v = np.concatenate(parts).astype(float)
    assert v.size == wire.OBS_N
    return v
