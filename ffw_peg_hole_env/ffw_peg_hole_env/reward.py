"""Peg-in-hole reward, computed from what both backends observe (Obs amps + poses).

Per step (all terms logged in `terms`):
    success   once (terminal), peg tip >= success_depth below the rim AND within
              success_lateral of the hole axis: r_success + w_quality *
              clip(1 - F_peak / f_hard, 0, 1), F_peak = the highest push-back since the
              peg could first touch the hole. The gentler the insertion, the more:
              0 N +1.5, 2 N +1.4, 4 N +1.3, 10 N +1.0 (each newton = 0.05)
    shape     potential-based progress, phi' - phi, phi = -w_shape * mean(lateral, tilt,
              depth-to-go), each normalized and capped at 1; depth below the rim only
              counts on the hole axis (not beside the block) -- sums to at most w_shape
              over an episode, so it guides without changing what is optimal
    force     -w_force * clip(F_pb / f_hard, 0, 1) every step: no dead zone, so less
              force is always better
    fail      once (terminal), only while the peg can touch the hole: F_pb > f_hard (jam --
              a stuck peg that keeps being pushed ends here too), or F_pb > f_rim for
              rim_hold ticks with the tip within rim_band of the rim (rim strike) --
              10 N hard (the live EdgeForce limit), 8 N near the rim. Value r_fail +
              r_fail_shallow * (share of success_depth not reached): the shallower the
              failure, the worse (jam at 33 of 34 mm ~ -0.5, stuck at 17 mm ~ -0.75, rim -1)

F_pb is the push-back on the peg along its axis from the RIGHT arm's currents --
the signal that separated clean from stalled pushes on the robot (left j7 is blind to
rim strikes, 2026-10-04 probes). Joint torque = effort.torque_from_amps (load-cell
driving K = K * eta), force by weighted LS through J^T at right_peg_site of the
IK-scene model (the same math as peg_hole_teach.EdgeForce), minus a reference held
while the peg cannot touch the hole (tip above the rim, or clear of the block across
the axis), so gravity / free-motion torques cancel.

Scale: on 106 recorded 2026-10-04 pushes this reads like the teach GUI's numbers
(peak median 12.6 vs 12.5 N, corr 0.93; free-air sd 0.46 N); the best manual pushes
peak at ~3-4 N.
"""
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from . import effort

IK_XML = (Path(__file__).resolve().parents[2] / "ffw_collision_checker" / "3rd_party"
          / "robotis_ffw" / "ffw_bg2_large_margin_no_gripper_joint.xml")


@dataclass
class RewardConfig:
    r_success: float = 1.0
    w_shape: float = 0.3
    lateral_scale: float = 0.05         # m: the reset's peg x/y range
    tilt_scale: float = 5.0             # deg: the reset's tilt range
    depth_scale: float = 0.074          # m: highest start (tip 4 cm above the rim) to success (3.4 cm in)
    w_quality: float = 0.5              # success bonus for a gentle insertion (0 N peak), 0 at f_hard
    w_force: float = 0.01               # per step at f_hard
    f_hard: float = 10.0                # N: jam -> episode ends (the live EdgeForce hard limit)
    f_rim: float = 8.0                  # N: near the rim -> rim strike, episode ends (EdgeForce's 4 N trips 21/50
                                        #    normal entries at 15 Hz -- their peak in the first 6 mm is ~4.9 N, p75 5.8;
                                        #    a real rim strike climbs to 17-23 N in sim)
    rim_band: float = 0.002             # m: tip at most this deep for the rim check -- a rim strike happens AT the
                                        # rim; deeper is chamfer / bore, where binding gets the machine's
                                        # intervention (7 N x 2) and only a 10 N jam ends it (was 6 mm, 2026-10-04)
    rim_hold: int = 2                   # ticks above f_rim (~0.13 s at 15 Hz; EdgeForce holds 0.1 s)
    r_fail: float = -0.5
    r_fail_shallow: float = -0.5        # extra, scaled by the depth still missing at the failure
    ref_window: int = 3                 # ticks above the rim the reference is the median of: short, so it is the
                                        # MOVING arm's (motor friction shifts the axial estimate ~6-8 N between
                                        # the static hover and the descent -- 8 ticks held the hover, 2026-10-04)
    ref_clear: float = 0.0005           # m: tip at least this far above the rim to update the reference


class PushBackForce:
    """Force on the peg along its axis [N] from the right arm's torques."""

    def __init__(self, xml=IK_XML, site="right_peg_site"):
        self.m = mujoco.MjModel.from_xml_path(str(xml))
        self.d = mujoco.MjData(self.m)
        self.site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, site)
        ids = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, f"arm_r_joint{i}") for i in range(1, 8)]
        self.qadr = [self.m.jnt_qposadr[j] for j in ids]
        self.dadr = [self.m.jnt_dofadr[j] for j in ids]
        self.lift = self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")]
        self.jp, self.jr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))

    def axial(self, q, lift, tau):
        self.d.qpos[self.qadr] = q
        self.d.qpos[self.lift] = lift
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        mujoco.mj_jacSite(self.m, self.d, self.jp, self.jr, self.site)
        J = self.jp[:, self.dadr]
        F, *_ = np.linalg.lstsq(J.T / effort.SIGMA[:, None], np.asarray(tau) / effort.SIGMA, rcond=None)
        return float((self.d.site_xmat[self.site].reshape(3, 3).T @ F)[0])


class PegHoleReward:
    def __init__(self, cfg=None, success_depth=0.034, success_lateral=0.002, block_halfwidth=0.0415):
        self.cfg = cfg or RewardConfig()
        self.success_depth, self.success_lateral, self.block_halfwidth = success_depth, success_lateral, block_halfwidth
        self.force = PushBackForce()
        self.reset()

    def reset(self):
        self.ref_buf, self.ref, self.phi = [], None, None
        self.rim_ticks = 0
        self.push_back, self.f_peak, self.f_axial = 0.0, 0.0, float("nan")

    def potential(self, ins):
        c = self.cfg
        lat = min(ins["lateral"] / c.lateral_scale, 1.0)
        tilt = min(ins["tilt_deg"] / c.tilt_scale, 1.0)
        depth = ins["depth"] if self.on_axis(ins) else min(ins["depth"], 0.0)
        togo = min(max(self.success_depth - depth, 0.0) / c.depth_scale, 1.0)
        return -c.w_shape * (lat + tilt + togo) / 3.0

    def missing(self, ins):
        """Share of success_depth not reached, 0..1."""
        return float(np.clip(1.0 - ins["depth"] / self.success_depth, 0.0, 1.0))

    def on_axis(self, ins):
        return ins["lateral"] <= self.success_lateral

    def clear_of_hole(self, ins):
        return ins["depth"] < -self.cfg.ref_clear or max(map(abs, ins["across"])) > self.block_halfwidth

    def push_back_force(self, ins, q, amps, lift):
        """F_pb [N] (> 0 = the peg is pushed back up its axis); updates the reference."""
        c = self.cfg
        f = self.f_axial = self.force.axial(q, lift, effort.torque_from_amps("right", amps))
        if self.clear_of_hole(ins):
            self.ref_buf = (self.ref_buf + [f])[-c.ref_window:]
            self.ref = float(np.median(self.ref_buf))
        self.push_back = 0.0 if self.ref is None else self.ref - f
        return self.push_back

    def __call__(self, ins, q, amps, lift):
        """-> (reward, success, failed, terms). Call once per tick, after the tick;
        call observe_reset() after a reset first so the potential starts there."""
        c = self.cfg
        f_pb = self.push_back_force(ins, q, amps, lift)
        success = ins["depth"] >= self.success_depth and self.on_axis(ins)
        touching = not self.clear_of_hole(ins)          # only then can push-back be contact (free-air
        near_rim = touching and ins["depth"] <= c.rim_band   # motion lags the reference: no fail there)
        self.rim_ticks = self.rim_ticks + 1 if near_rim and f_pb > c.f_rim else 0
        rim_strike = self.rim_ticks >= c.rim_hold
        failed = (not success) and touching and (f_pb > c.f_hard or rim_strike)
        if touching:
            self.f_peak = max(self.f_peak, f_pb)
        quality = float(np.clip(1.0 - self.f_peak / c.f_hard, 0.0, 1.0))
        phi = self.potential(ins)
        terms = {"success": c.r_success + c.w_quality * quality if success else 0.0,
                 "shape": 0.0 if self.phi is None else phi - self.phi,
                 "force": -c.w_force * float(np.clip(f_pb / c.f_hard, 0.0, 1.0)),
                 "fail": c.r_fail + c.r_fail_shallow * self.missing(ins) if failed else 0.0}
        self.phi = phi
        return float(sum(terms.values())), bool(success), bool(failed), {
            **terms, "push_back_N": f_pb, "peak_push_back_N": self.f_peak, "rim_strike": bool(failed and rim_strike)}

    def observe_reset(self, ins, q, amps, lift):
        self.reset()
        self.push_back_force(ins, q, amps, lift)
        self.phi = self.potential(ins)
