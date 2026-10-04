"""Pose helpers shared by every backend.

Poses are 4x4 homogeneous matrices in base_link (the sim world frame is the
robot base). A command delta is 6 numbers (dx, dy, dz, droll, dpitch, dyaw):
translation added in base_link, rotation applied in base_link as
R_new = R_delta @ R with R_delta from intrinsic X-Y-Z Euler angles (the
convention proposed with ControlCmdDelta; for the small per-tick deltas it is
the same as a rotation vector to first order).
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# Tool sites have x along the insertion axis (up, out of the hole). The
# "hole frame" used for sampling has z along that axis instead:
# hole-frame x, y, z = site y, z, x.
SITE_TO_ZUP = np.array([[0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])


def T_from(p, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def rot(T):
    return T[:3, :3]


def pos(T):
    return T[:3, 3]


def zup(T_site):
    """Site pose -> the same point with the z-up hole-frame axes."""
    return T_from(pos(T_site), rot(T_site) @ SITE_TO_ZUP)


def apply_delta(T, delta):
    """Apply a (dx, dy, dz, droll, dpitch, dyaw) base_link delta to pose T."""
    d = np.asarray(delta, dtype=float)
    out = T.copy()
    out[:3, 3] = pos(T) + d[:3]
    out[:3, :3] = Rot.from_euler("XYZ", d[3:6]).as_matrix() @ rot(T)
    return out


def delta_between(T_from_, T_to):
    """The base_link delta that takes T_from_ to T_to (inverse of apply_delta)."""
    R_d = rot(T_to) @ rot(T_from_).T
    return np.concatenate([pos(T_to) - pos(T_from_), Rot.from_matrix(R_d).as_euler("XYZ")])


def clip_delta(delta, max_trans, max_rot):
    """Per-axis clip: each translation component to +-max_trans [m], each
    rotation component to +-max_rot [rad] (agent action +-1 on every axis maps
    to exactly the cap on that axis)."""
    d = np.asarray(delta, dtype=float).copy()
    d[:3] = np.clip(d[:3], -max_trans, max_trans)
    d[3:6] = np.clip(d[3:6], -max_rot, max_rot)
    return d


def pose_error(T_a, T_b):
    """(translation error m, rotation error rad) between two poses."""
    return (float(np.linalg.norm(pos(T_a) - pos(T_b))),
            float(np.linalg.norm(Rot.from_matrix(rot(T_b) @ rot(T_a).T).as_rotvec())))


class SafetyBox:
    """Safe range for one pose (an EE or its tool point), like the teleop limit
    profile but in any frame: translation bounds along the axes of `T_frame`
    (so the primary axis can be e.g. the hole's insertion axis instead of a
    base_link axis), and optionally a rotation limit: the deviation from
    `R_ref` as a rotation vector expressed in the frame's axes, clamped per axis.

    lo, hi  : (3,) m, position in the frame (origin = frame origin)
    rot_lim : (3,) rad per frame axis, or None for no rotation limit
    R_ref   : (3,3) reference orientation in base_link (the "aligned" pose)
    """

    def __init__(self, T_frame, lo, hi, rot_lim=None, R_ref=None):
        self.T = np.asarray(T_frame, dtype=float)
        self.Ti = np.linalg.inv(self.T)
        self.lo, self.hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
        self.rot_lim = None if rot_lim is None else np.asarray(rot_lim, dtype=float)
        self.R_ref = np.eye(3) if R_ref is None else np.asarray(R_ref, dtype=float)

    def where(self, T):
        """(position in the frame, rotation deviation from R_ref in frame axes)."""
        p = (self.Ti @ np.r_[pos(T), 1.0])[:3]
        dev_base = Rot.from_matrix(rot(T) @ self.R_ref.T).as_rotvec()        # base-frame rotvec
        return p, rot(self.T).T @ dev_base

    def clamp(self, T):
        """T limited to the box. Returns (T_clamped, clamped?)."""
        p, dev = self.where(T)
        pc = np.clip(p, self.lo, self.hi)
        out = T.copy()
        hit = not np.allclose(pc, p)
        out[:3, 3] = (self.T @ np.r_[pc, 1.0])[:3]
        if self.rot_lim is not None:
            dc = np.clip(dev, -self.rot_lim, self.rot_lim)
            if not np.allclose(dc, dev):
                hit = True
                out[:3, :3] = Rot.from_rotvec(rot(self.T) @ dc).as_matrix() @ self.R_ref
        return out, hit
