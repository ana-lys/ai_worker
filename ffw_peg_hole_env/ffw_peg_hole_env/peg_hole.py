"""Peg-in-hole episode logic on top of a backend (sim now, real later).

reset(seed)  sample the hole and the peg start in the hole frame (z = insertion
             axis, up), place both arms there, latch the command references.
step(right_delta, left_delta)
             apply 6-DOF base_link deltas (clipped per tick) to the commanded
             **EE** poses (the IK end-effector, as on the robot), solve IK,
             advance one agent tick, return (obs, reward, terminated,
             truncated, info).

Sampling and the success check use the peg/hole tool frames; commands use
the EE. The backend provides: site_pose(side) (EE), tool_pose(side),
ee_to_tool[side], command(side, T_ee), tick(), teleport({side: T_ee}),
reachable(side, T_ee), joint_state(side), peg_hole_force(),
tool_geometry() -> (peg tip, hole rim) along the tool x.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import effort
from .geometry import SITE_TO_ZUP, SafetyBox, T_from, apply_delta, clip_delta, pos, rot, zup
from .reward import PegHoleReward, RewardConfig


@dataclass
class PegHoleConfig:
    hz: float = 15.0
    roll_deg: float = 90.0                  # peg site roll about the hole axis when aligned
    hole_base_up: float = 0.10              # nominal hole = start hole site raised this much (base z), if hole_nominal is None
    # nominal hole site pose (base_link = MJCF world): position xyz + quat xyzw. Default: the most robust
    # centre of ffw_collision_checker/scripts/survey_peg_hole_region.py (2026-10-02, lift -0.30 m, wide
    # lattice): the whole +-5 cm x 0..-2 cm box stays IK-reachable even shifted 9.9 cm; right joints stay
    # >= 23.5 deg from their limits through every push of the box.
    hole_nominal: tuple = (0.5424, 0.0946, 0.8548, -0.215676, -0.637369, -0.20768, 0.71001)
    hole_xy: float = 0.05                   # hole sample, hole frame: x/y +-
    hole_z: tuple = (-0.02, 0.0)            #                          z range
    peg_xy: float = 0.05                    # peg start, relative to the sampled hole: x/y +-
    peg_tip_z: tuple = (0.02, 0.04)         #   peg tip height above the hole rim
    peg_tilt_deg: float = 5.0               #   +- roll/pitch/yaw tilt from aligned
    max_trans: float = 0.01 * 2 / 3         # per-tick, per-axis clip [m]: 6.67 mm (0.10 m/s at 15 Hz)
    max_rot: float = np.radians(2.0)        # per-tick, per-axis clip [rad]: 2 deg (30 deg/s at 15 Hz)
    success_depth: float = 0.034            # peg tip this far below the rim = inserted (the robot's full 35 mm
                                            #     push - 1 mm; recorded jams sit at 16-19 mm, so 15 mm scored them
                                            #     as successes. The sim walls are 26 mm long: deeper is free) ...
    success_lateral: float = 0.002          # ... and on the hole axis (walls allow 0.5 mm; beside the block the
                                            #     tip can drop below the rim plane without being in the hole)
    block_halfwidth: float = 0.0415         # peg centre farther than this across the hole axis = clear of the
                                            #     hole block (26 mm block + 15.5 mm peg half-width), no contact
    max_contact_force: float = 300.0        # sim only: true peg-hole contact above this also ends the episode
    reward: RewardConfig = field(default_factory=RewardConfig)   # reward.py; its f_hard is the failure both backends share
    max_steps: int = 300                    # 20 s at 15 Hz
    # Safe range for both arms (every step clamps the commanded pose; the policy cannot integrate past
    # it). safety_frame: "hole" = the nominal hole frame (z = insertion axis up, x/y across it -- the
    # reset box's axes), "hole_live" = this episode's sampled hole frame, "base" = base_link axes with the
    # origin at the nominal hole, "pose" = safety_pose (xyz + quat xyzw, base_link). safety_point: "tool"
    # clamps the peg / hole tool point, "ee" the EE site. Bounds per arm: x lo, hi, y lo, hi, z lo, hi
    # [m, in the frame]; rotation: max deviation [deg] about each frame axis from the aligned orientation
    # (the hole frame for the left tool, the aligned peg for the right), None = no rotation limit.
    safety: bool = True
    safety_frame: str = "hole"
    safety_pose: tuple = None
    safety_point: str = "tool"
    safety_right: tuple = (-0.13, 0.13, -0.13, 0.13, -0.05, 0.12)
    safety_rot_right_deg: tuple = (15.0, 15.0, 15.0)
    safety_left: tuple = (-0.07, 0.07, -0.07, 0.07, -0.04, 0.02)
    safety_rot_left_deg: tuple = (5.0, 5.0, 5.0)
    reset_tries: int = 20
    reset_tol: tuple = (0.002, np.radians(1.0))
    stats: dict = field(default_factory=dict)


def rel_aligned(s, roll_deg):
    """Peg site pose in the hole site frame: on the axis at height s, rolled."""
    return T_from([s, 0.0, 0.0], Rot.from_euler("x", roll_deg, degrees=True).as_matrix())


class PegHoleTask:
    def __init__(self, backend, cfg=None):
        self.b = backend
        self.cfg = cfg or PegHoleConfig()
        self.tip, self.rim = backend.tool_geometry()
        if self.cfg.hole_nominal is not None:
            hn = self.cfg.hole_nominal
            self.hole_nominal = T_from(hn[:3], Rot.from_quat(hn[3:]).as_matrix())
        else:
            self.hole_nominal = backend.tool_pose("left").copy()
            self.hole_nominal[2, 3] += self.cfg.hole_base_up
        self.T_cmd = {s: backend.site_pose(s) for s in ("left", "right")}
        self.steps = 0
        self.hole_site = self.hole_nominal
        self.safety = self.build_safety(self.hole_nominal)
        self.reward = PegHoleReward(self.cfg.reward, self.cfg.success_depth, self.cfg.success_lateral,
                                    self.cfg.block_halfwidth)

    # --- safe range ---------------------------------------------------------------
    def build_safety(self, hole_site):
        """SafetyBox per arm for this config (see PegHoleConfig.safety_*), in the tool
        or EE space per safety_point; hole_site = the hole the frame/reference uses."""
        c = self.cfg
        if not c.safety:
            return {}
        frame_hole = self.hole_nominal if c.safety_frame == "hole" else hole_site
        if c.safety_frame in ("hole", "hole_live"):
            T_f = zup(frame_hole)
        elif c.safety_frame == "base":
            T_f = T_from(pos(self.hole_nominal), np.eye(3))
        elif c.safety_frame == "pose":
            T_f = T_from(c.safety_pose[:3], Rot.from_quat(c.safety_pose[3:]).as_matrix())
        else:
            raise ValueError(f"safety_frame {c.safety_frame!r}")
        ref_tool = {"left": rot(frame_hole),
                    "right": rot(frame_hole @ rel_aligned(0.0, c.roll_deg))}
        boxes = {}
        for side, b, r in (("left", c.safety_left, c.safety_rot_left_deg),
                           ("right", c.safety_right, c.safety_rot_right_deg)):
            R_ref = ref_tool[side]
            if c.safety_point == "ee":
                R_ref = R_ref @ rot(np.linalg.inv(self.b.ee_to_tool[side]))
            boxes[side] = SafetyBox(T_f, b[0::2], b[1::2], None if r is None else np.radians(r), R_ref)
        return boxes

    def clamp_cmd(self, side):
        """Limit the commanded EE pose of `side` to its safe range. Returns clamped?"""
        box = self.safety.get(side)
        if box is None:
            return False
        if self.cfg.safety_point == "tool":
            T_tool, hit = box.clamp(self.T_cmd[side] @ self.b.ee_to_tool[side])
            self.T_cmd[side] = self.to_ee(side, T_tool)
        else:
            self.T_cmd[side], hit = box.clamp(self.T_cmd[side])
        return hit

    # --- geometry ---------------------------------------------------------------
    def peg_site_height(self, tip_above_rim):
        """Peg-site height s (along the hole axis) that puts the peg tip this far above the rim."""
        return self.rim + tip_above_rim - self.tip

    def insertion_state(self):
        """True peg pose in the hole: height s, lateral offset (and its two components
        across the axis), tip depth below the rim, tilt of the peg axis from the hole axis."""
        T_rel = np.linalg.inv(self.b.tool_pose("left")) @ self.b.tool_pose("right")
        s = float(T_rel[0, 3])
        lateral = float(np.hypot(T_rel[1, 3], T_rel[2, 3]))
        depth = self.rim - (s + self.tip)
        tilt = float(np.degrees(np.arccos(np.clip(T_rel[0, 0], -1.0, 1.0))))
        return {"height": s, "lateral": lateral, "across": (float(T_rel[1, 3]), float(T_rel[2, 3])),
                "depth": depth, "tilt_deg": tilt, "T_rel": T_rel}

    def sample(self, rng):
        c = self.cfg
        H_nom = zup(self.hole_nominal)
        dh = np.array([rng.uniform(-c.hole_xy, c.hole_xy), rng.uniform(-c.hole_xy, c.hole_xy),
                       rng.uniform(*c.hole_z)])
        H = H_nom.copy()
        H[:3, 3] = pos(H_nom) + rot(H_nom) @ dh
        hole_site = T_from(pos(H), rot(H) @ SITE_TO_ZUP.T)
        tip_h = rng.uniform(*c.peg_tip_z)
        aligned = hole_site @ rel_aligned(self.peg_site_height(tip_h), c.roll_deg)
        off = np.array([rng.uniform(-c.peg_xy, c.peg_xy), rng.uniform(-c.peg_xy, c.peg_xy), 0.0])
        tilt = Rot.from_euler("xyz", rng.uniform(-c.peg_tilt_deg, c.peg_tilt_deg, 3), degrees=True).as_matrix()
        R_tilt_base = rot(H) @ tilt @ rot(H).T                 # tilt expressed about hole-frame axes
        peg = T_from(pos(aligned) + rot(H) @ off, R_tilt_base @ rot(aligned))
        return hole_site, peg, {"hole_offset": dh, "peg_offset": off, "peg_tip_above_rim": tip_h,
                                "peg_tilt_deg": np.degrees(Rot.from_matrix(tilt).as_euler("xyz"))}

    def to_ee(self, side, T_tool):
        return T_tool @ np.linalg.inv(self.b.ee_to_tool[side])

    # --- episode ------------------------------------------------------------------
    def reset(self, seed=None):
        c = self.cfg
        rng = np.random.default_rng(seed)
        for attempt in range(1, c.reset_tries + 1):
            hole_site, peg, info = self.sample(rng)
            # the insertion itself must be reachable, not only the start
            # (e.g. right J2 hits its 0 deg limit for some hole samples)
            insert_poses = [hole_site @ rel_aligned(self.peg_site_height(h), c.roll_deg)
                            for h in (c.peg_tip_z[1], 0.0, -c.success_depth - 0.005)]
            if not all(self.b.reachable("right", self.to_ee("right", T)) for T in insert_poses):
                continue
            pe, re = self.b.teleport({"left": self.to_ee("left", hole_site), "right": self.to_ee("right", peg)})
            if pe < c.reset_tol[0] and re < c.reset_tol[1] and self.b.peg_hole_force() == 0.0:
                break
        else:
            raise RuntimeError(f"reset: no reachable sample in {c.reset_tries} tries")
        self.hole_site = hole_site
        self.T_cmd = {"left": self.to_ee("left", hole_site), "right": self.to_ee("right", peg)}
        if c.safety and c.safety_frame == "hole_live":
            self.safety = self.build_safety(hole_site)
        self.steps = 0
        info.update({"seed": seed, "reset_attempts": attempt, "reset_err_mm": pe * 1000})
        obs = self.observe()
        self.reward.observe_reset(obs["insertion"], *self.right_effort(obs))
        return obs, info

    def step(self, right_delta, left_delta=None):
        c = self.cfg
        self.T_cmd["right"] = apply_delta(self.T_cmd["right"], clip_delta(right_delta, c.max_trans, c.max_rot))
        if left_delta is not None:
            self.T_cmd["left"] = apply_delta(self.T_cmd["left"], clip_delta(left_delta, c.max_trans, c.max_rot))
        clamped = {side: self.clamp_cmd(side) for side in ("left", "right")}
        self.b.command_all(self.T_cmd)
        self.b.tick()
        self.steps += 1
        obs = self.observe()
        reward, success, failed, terms = self.reward(obs["insertion"], *self.right_effort(obs))
        failed = failed or (not success and obs["contact_force"] > c.max_contact_force)
        terminated = bool(success or failed)
        truncated = bool(not terminated and self.steps >= c.max_steps)
        obs["push_back"] = terms["push_back_N"]
        info = {"success": bool(success), "failed": bool(failed), "steps": self.steps,
                "reward_terms": terms, "safety_clamped": {k: bool(v) for k, v in clamped.items()}}
        return obs, reward, terminated, truncated, info

    def observe(self):
        obs = {"insertion": self.insertion_state(), "contact_force": self.b.peg_hole_force(),
               "lift": float(self.b.d.qpos[self.b.lift_qadr])}
        for side in ("left", "right"):
            q, dq, tau = self.b.joint_state(side)
            obs[side] = {"ee": self.b.site_pose(side), "tool": self.b.tool_pose(side),
                         "cmd": self.T_cmd[side].copy(), "q": q, "dq": dq, "effort": tau,
                         "amps": effort.amps_from_torque(side, tau)}        # what Obs carries
        return obs

    @staticmethod
    def right_effort(obs):
        """(q, amps, lift) of the right arm: the reward's inputs, all in Obs."""
        r = obs["right"]
        return r["q"], r["amps"], obs["lift"]
