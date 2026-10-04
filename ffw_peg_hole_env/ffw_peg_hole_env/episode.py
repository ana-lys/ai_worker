"""Per-tick episode logic for the real robot: frame state, termination, machine intervention.

No ROS here. The server measures, calls tick() once per 15 Hz frame after the robot
moved, publishes the returned tag, and does what `mode` says next:

  "policy"     apply the policy's delta
  "pull_out"   machine: move the peg up the hole axis (pull_out_delta)
  "trace_back" machine: move the peg to self.restore (toward_delta), then report
               arrived=True on the tick it gets there
  "reset"      episode over: run the reset, then start() the next one

Frame states (wire.FS_*):
  POLICY        the policy drove this tick
  INTERVENTION  the machine drove it (pull out along the axis, then back to the last
                pose the peg had near the hole top); the policy resumes after it,
                same episode; at most max_interventions per episode
  TERMINATED    last frame (reason wire.TR_*): success / jam / rim (reward.py rules),
                blocked, timeout, safety (a live stop of the robot), interventions
                (one more was needed than allowed)

Interaction zone (user, 2026-10-04): within zone_radius of the (calibrated) hole axis
the peg and the hole may interact freely -- inside the hole block's footprint (~+-4 cm
for the peg centre) the peg can only land on the block top / chamfer / rim, which the
force rules price and limit. Outside it, the tip going below the rim means the peg is
heading down BESIDE the block (the 2026-10-04 incident): the machine halts it and
intervenes.

Intervention triggers (only while the peg can touch the hole): the tip below the rim
outside the zone; push-back above f_intervene (below reward.f_hard, so the machine
steps in before the jam fail); blocked progress; or the left j7 current changed more
than j7_intervene since the peg was last clear.

Safety (TERMINATED SAFETY, any mode): the hole pushed sideways more than
hole_shift_max since the peg was last clear, or a live stop of the robot.

"Can touch the hole" on the real robot = the tip within touch_clear above the rim or
lower, at ANY offset: the peg's side rubs the block / the left gripper when it goes
down beside it (2026-10-04 incident), so the sim's block-footprint shortcut is off
(footprint=None).

Thresholds from the 2026-10-04 recordings, measured from the last tick the peg was
clear: clean pushes moved the hole sideways p95 2.2 / max 4.2 mm and changed left j7
p95 ~200 mA; stalls 300-460 mA; the incident's reset pushed the hole 88 mm.
"""
from dataclasses import dataclass

import numpy as np

from . import wire
from .geometry import clip_delta, delta_between
from .reward import PegHoleReward, RewardConfig


@dataclass
class EpisodeConfig:
    timeout_s: float = 20.0
    f_intervene: float = 6.0            # N push-back -> machine steps in (fail is at reward f_hard = 10 N)
    zone_radius: float = 0.02           # m: free interaction within this of the axis; outside + below the rim -> machine
    j7_intervene: float = 300.0         # mA left j7 change since last clear -> machine steps in
    hole_shift_max: float = 0.006       # m hole pushed sideways since last clear -> SAFETY
    touch_clear: float = 0.001          # m: tip above the rim by more than this = cannot touch (real robot)
    footprint: float = None             # m peg-centre block half-width for the sim shortcut; None = real robot
    max_interventions: int = 3
    clear: float = 0.005                # m: pull out until the tip is this far above the rim
    top_lo: float = 0.002               # m: "near the hole top" = tip top_lo..top_hi above the rim,
    top_hi: float = 0.015               #    within lateral_top of the axis (the restore pose)
    lateral_top: float = 0.003
    max_trans: float = 0.01 * 2 / 3     # machine per-tick caps = the policy's (6.67 mm, 2 deg)
    max_rot: float = np.radians(2.0)


class EpisodeMachine:
    def __init__(self, cfg=None, reward_cfg=None, success_depth=0.034):
        self.cfg = cfg or EpisodeConfig()
        self.reward = PegHoleReward(reward_cfg or RewardConfig(), success_depth,
                                    block_halfwidth=self.cfg.footprint if self.cfg.footprint is not None else np.inf)
        self.episode_id, self.step, self.mode = 0, 0, "idle"

    def start(self, episode_id, ins, q, amps, lift, T_peg):
        """New episode, right after a reset: (ins, q, amps, lift) as for reward.py,
        T_peg = the peg's commanded EE pose (base_link)."""
        self.episode_id, self.step, self.t = episode_id, 0, 0.0
        self.mode, self.interventions, self.restore = "policy", 0, None
        self.reward.observe_reset(ins, q, amps, lift)
        self._remember(ins, T_peg)

    def _remember(self, ins, T_peg):
        c = self.cfg
        if c.top_lo <= -ins["depth"] <= c.top_hi and ins["lateral"] <= c.lateral_top:
            self.restore = T_peg.copy()

    def can_touch(self, ins):
        return ins["depth"] >= -self.cfg.touch_clear and not self.reward.clear_of_hole(ins)

    def _trigger(self, ins, f_pb, blocked, dj7):
        c = self.cfg
        if not self.can_touch(ins):
            return False
        return (f_pb > c.f_intervene or blocked or dj7 > c.j7_intervene
                or (ins["depth"] > 0.0 and ins["lateral"] > c.zone_radius))

    def tick(self, dt, ins, q, amps, lift, T_peg, blocked=False, safety=False, arrived=False,
             hole_shift=0.0, dj7=0.0):
        """One frame, after the robot moved. hole_shift [m] = the hole's sideways move and
        dj7 [mA] = |left j7 current change|, both since the peg was last clear (the server
        keeps that reference). -> dict(frame_state, reason, reward, mode, terms)."""
        c = self.cfg
        self.step += 1
        self.t += dt
        reward, success, failed, terms = self.reward(ins, q, amps, lift)
        f_pb = terms["push_back_N"]
        driving = self.mode                                     # who moved the robot this tick
        state = wire.FS_INTERVENTION if driving in ("pull_out", "trace_back") else wire.FS_POLICY
        reason = wire.TR_NONE
        if driving == "policy":
            self._remember(ins, T_peg)
        if safety or hole_shift > c.hole_shift_max:
            reason = wire.TR_SAFETY
        elif success:
            reason = wire.TR_SUCCESS
        elif failed and driving == "policy":                    # the machine pulling out may read force too
            reason = wire.TR_RIM if terms["rim_strike"] else wire.TR_JAM
        elif self.t >= c.timeout_s - 1e-6:                     # summed dt drifts below the exact value
            reason = wire.TR_TIMEOUT
        elif driving == "policy" and self._trigger(ins, f_pb, blocked, dj7):
            if self.interventions >= c.max_interventions:
                reason = wire.TR_BLOCKED if blocked else wire.TR_INTERVENTIONS
            else:
                self.interventions += 1
                self.mode = "pull_out"
        elif driving == "pull_out" and -ins["depth"] >= c.clear:
            self.mode = "trace_back"
        elif driving == "trace_back" and arrived:
            self.mode = "policy"
        if reason != wire.TR_NONE:
            state, self.mode = wire.FS_TERMINATED, "reset"
            if reason not in (wire.TR_SUCCESS,) and terms["fail"] == 0.0:
                # ended by a rule reward.py doesn't price: price it like its fail
                miss = self.reward.missing(ins)
                terms["fail"] = self.reward.cfg.r_fail + self.reward.cfg.r_fail_shallow * miss \
                    if reason != wire.TR_TIMEOUT else 0.0
                reward = float(sum(terms[k] for k in ("success", "shape", "force", "fail")))
        return {"frame_state": state, "reason": reason, "reward": reward, "mode": self.mode,
                "terms": terms, "step": self.step, "interventions": self.interventions}

    # --- machine motion (per-tick deltas, capped like the policy's) --------------------------
    def pull_out_delta(self, hole_axis):
        """Straight up the hole axis (unit vector, base_link) by one capped step."""
        return np.concatenate([np.asarray(hole_axis) * self.cfg.max_trans, np.zeros(3)])

    def toward_delta(self, T_cmd, T_target):
        """One capped step from the commanded pose toward T_target (6-DOF base_link delta)."""
        return clip_delta(delta_between(T_cmd, T_target), self.cfg.max_trans, self.cfg.max_rot)
