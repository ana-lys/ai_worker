#!/usr/bin/env python3
"""HIL-SERL peg-in-hole server for the REAL robot (ffw_peg_hole_env/SERL_REAL_PLAN.md).

Publishes one Frame per 15 Hz tick on base+0: obs (what a deployed actor measures: 25
joints x [pos, vel, effort], both IK EE poses + velocities, both arms' margins to their
peg_hole_safe_real limit box), priv (everything the server derives: reward terms and
return, the push-back force estimate, task geometry, env flags, the gateway's full Obs)
and the tag (episode_id, step, frame_state, reason, reward, applied action); listens on
base+1 for ControlCmdDelta / EnvCmd. Layout: ffw_peg_hole_env/wire.py, FRAME.md.

Without --allow-motion: IDLE frames only, subscribe-only (a wire check).
With --allow-motion (slices 3-4): episodes, all automatic after the first EnvCmd RESET
(or --autostart):
  RESET frames     peg straight up the hole axis to CLEAR first (watching the hole), then
                   Teach.hw_reset: new random reach-checked pose, move there; a lag trip
                   or a pushed hole -> FAULT at once (no retries)
  POLICY frames    each ControlCmdDelta (right arm, base_link, applied to the commanded
                   IK EE pose = the pose Obs.ee reports, clipped 6.67 mm / 2 deg per
                   axis, clamped to a box around the hole and, farther than 2 cm from the
                   calibrated axis, to above the rim); no delta for 0.2 s = hold
  INTERVENTION     the machine: pull the peg out along the hole axis, trace back to the
                   last policy pose near the hole top, then the policy again (triggers: the tip
                   below the rim > 2 cm off the axis, push-back > 6 N, blocked, left j7)
  TERMINATED       episode.EpisodeMachine (reward.py rules, blocked, timeout) or a
                   live stop (hard edge force, left j7 hard current); then RESET again
EnvCmd PAUSE: finish nothing new after the current episode (IDLE); RESUME: go on;
ABORT: end the episode now (TERMINATED ABORT), then IDLE. Every tick is also recorded
by the teach tool's recorder under --out.

  source ROS; .venv/bin/python peg_hole_serl_server.py --port-base 7601            # wire check
  source ROS; .venv/bin/python peg_hole_serl_server.py --allow-motion --continuous   # real episodes
"""
import argparse
import json
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import rclpy
import rclpy.signals
from rclpy.executors import SingleThreadedExecutor
import zmq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "ffw_peg_hole_env"))
sys.path.insert(0, str(HERE.parents[1] / "ffw_zmqinterface"))
from ffw_peg_hole_env import effort, images, observation, wire  # noqa: E402
from ffw_peg_hole_env.episode import EpisodeMachine  # noqa: E402
from ffw_peg_hole_env.geometry import SafetyBox, apply_delta, clip_delta, delta_between, pose_error  # noqa: E402
from ffw_zmqinterface import gateway_node as gwn, protocol as gw  # noqa: E402
import peg_hole_random as phr  # noqa: E402
import peg_hole_stroke as phs  # noqa: E402
import peg_hole_teach as pht  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

HZ = 15.0
RIGHT = [f"arm_r_joint{i}" for i in range(1, 8)]
MAX_TRANS, MAX_ROT = 0.01 * 2 / 3, np.radians(2.0)     # per tick, per axis: the policy's +-1
WATCHDOG_S = 0.2
POLICY_WAIT = 0.04                                      # s a policy tick waits for the client's reply to the last frame
# workspace box for the peg tool point, hole tool frame (x = insertion axis up, from the
# hole tool origin): along x TOP - push - 3 mm .. TOP + 6 cm, across +-6 cm; tilt +-10 deg
BOX_ACROSS, BOX_UP, BOX_TILT = 0.06, 0.06, np.radians(10.0)
# interaction zone: farther than ZONE from the calibrated axis the peg tool point stays FLOOR
# above ON_TOP, so it can never head down beside the hole block (2026-10-04 incident); inside
# it the peg may touch the block top / chamfer / rim (the force rules and the gear guard limit
# that). episode.EpisodeMachine halts + intervenes if it gets below the rim outside anyway.
ZONE, FLOOR = 0.025, 0.005                              # FLOOR covers the arm's ~0.2 s lag (2.4 mm overshoot seen live)
RETRACT_SPEED = 0.03                                    # m/s pulling the peg out of the bore (insertion: 0.01)
GOOD_CLEAR = 0.003                                      # m: a "good state" has the tip at least this far above the rim
QUICK_SETTLE = 0.1                                      # s pause after an intermediate reset move (teach: 0.3)
HOLE_SHIFT_MAX = 0.006                                  # m: hole pushed sideways -> stop (resets too)
# Gear guard, every control tick in every phase (episode, intervention, retract, reset):
# left j7 current change from its unloaded reference > J7_HARD at once, or > J7_SUSTAIN for
# J7_HOLD s. Recordings: clean pushes peak <= 400 mA and never stay > 300 mA longer than
# 0.30 s; the 2026-10-04 reset drag pinned j7 at 1.5 A for ~40 s (this trips at 450 mA,
# ~0.05 s into the drag). During a retract only a RISE of the load trips (unloading is fine).
J7_HARD, J7_SUSTAIN, J7_HOLD, J7_RISE = 450.0, 300.0, 0.35, 150.0
OFFSET_MODEL = HERE.parent / "config" / "peg_hole_offset_model.json"


# The scripted expert's settings (peg_hole_demo_recorder.py exposes them as options; the 100 demos
# used these values). Also the machine intervention in --failure-mode intervene.
EXPERT_CFG = types.SimpleNamespace(hover=0.005, hover_speed=0.03, insert_speed=0.01, settle_ticks=5,
                                   align_tol=0.0004, hole_still=0.0002, align_gain=0.15, align_timeout=3.0)


class Expert:
    """Over the (offset) hole axis at hover; wait there until the hole has stopped moving and the
    MEASURED peg is on the calibrated axis (closing the loop on the arm's tracking error, which
    left ~1 mm at the rim open-loop); then straight down the axis, keeping that correction.
    2026-10-04 demos: open-loop first tries reached the rim with the hole still settling
    (0.6 mm/s, retries 0.08) and the peg +-1 mm off its command -- most second tries worked."""

    def __init__(self, srv, a, off_mm):
        """a: EXPERT_CFG-like (hover, hover_speed, insert_speed, settle_ticks, align_tol, hole_still,
        align_gain, align_timeout); off_mm: the peg-tool (y, z) offset to insert at."""
        self.srv, self.a = srv, a
        self.off_mm = np.asarray(off_mm, float)
        c, s_ = np.cos(np.radians(pht.ROLL)), np.sin(np.radians(pht.ROLL))
        self.R2 = np.array([[c, -s_], [s_, c]])              # peg-tool (y, z) -> hole-tool (y, z)
        self.corr = np.zeros(2)                             # hole-tool frame correction [m]
        self.phase, self.settled, self.n_int, self.k_align = "approach", 0, srv.em.interventions, 0
        self.s = pht.TOP + a.hover
        self.hole_hist = []

    def target(self, s):
        t = self.srv.t
        off = self.off_mm / 1000.0 - self.R2.T @ self.corr  # command shifted against the measured error
        return t.peg_target(s, phs.T_from([0.0, *off], np.eye(3)))[0] @ t.st.arms["right"].map

    def measured(self):
        """(peg lateral error from the calibrated axis in the hole-tool frame (y, z) [m], hole speed [m/s])."""
        srv = self.srv
        H = srv.io.ee("left") @ pht.HOLE_TOOL
        P = np.linalg.inv(H) @ srv.io.ee("right") @ pht.PEG_TOOL
        self.hole_hist = (self.hole_hist + [H[:3, 3].copy()])[-6:]
        v = np.linalg.norm(self.hole_hist[-1] - self.hole_hist[0]) * HZ / (len(self.hole_hist) - 1) \
            if len(self.hole_hist) > 1 else np.inf
        return P[1:3, 3] - srv.axis_c, v

    def delta(self):
        srv, a = self.srv, self.a
        e, v_hole = self.measured()
        if srv.em.interventions != self.n_int:               # the machine stepped in: start over from hover
            self.n_int, self.phase, self.settled, self.s = srv.em.interventions, "approach", 0, pht.TOP + a.hover
        if self.phase in ("approach", "align"):
            tgt = self.target(pht.TOP + a.hover)
            if self.phase == "approach":
                err = pose_error(srv.T_cmd, tgt)
                self.settled = self.settled + 1 if err[0] < 2e-4 and err[1] < np.radians(0.2) else 0
                if self.settled >= a.settle_ticks:
                    self.phase, self.settled, self.k_align = "align", 0, 0
            else:
                self.k_align += 1
                self.corr = np.clip(self.corr + a.align_gain * e, -0.003, 0.003)
                ok = np.linalg.norm(e) < a.align_tol and v_hole < a.hole_still
                self.settled = self.settled + 1 if ok else 0
                if self.settled >= a.settle_ticks or self.k_align >= a.align_timeout * HZ:
                    print(f"    aligned: peg {np.linalg.norm(e) * 1000:.2f} mm off the axis, hole "
                          f"{v_hole * 1000:.2f} mm/s, correction {np.round(self.corr * 1000, 2)} mm, "
                          f"{self.k_align / HZ:.1f} s{' (timeout)' if self.settled < a.settle_ticks else ''}")
                    self.phase = "insert"
            return clip_delta(delta_between(srv.T_cmd, tgt), a.hover_speed / HZ, MAX_ROT)
        self.s = max(self.s - a.insert_speed / HZ, pht.TOP - srv.t.a.push - 0.002)
        return clip_delta(delta_between(srv.T_cmd, self.target(self.s)), MAX_TRANS, MAX_ROT)



class HoleMoved(Exception):
    pass


class GuardTrip(Exception):
    pass


def gateway_obs(io):
    """Gateway-identical gw.Obs from /joint_states + /ik_solver/achieved_ee_pose_{r,l},
    or None until both are in."""
    raw = io.raw_js
    sites = [io.latest_site("right"), io.latest_site("left")]          # Obs ee0 = right, ee1 = left
    if raw is None or any(s is None for s in sites):
        return None
    js = type("JS", (), {"name": raw[1], "position": raw[2], "velocity": raw[3], "effort": raw[4]})
    o = gw.Obs()
    o.joint_pos, o.joint_vel, o.joint_effort = gwn._reorder_joint_block(js)      # effort -> A for the arms
    ee = []
    for T in sites:
        x, y, z, w = Rot.from_matrix(T[:3, :3]).as_quat()
        ee.append((*T[:3, 3], *gw.quat_to_rpy(w, x, y, z)))
    o.ee = (tuple(float(v) for v in ee[0]), tuple(float(v) for v in ee[1]))
    pos = dict(zip(raw[1], raw[2]))
    o.gripper = tuple(gwn._normalize_grip(pos.get(gwn._GRIP_JOINT[i]), i) for i in (0, 1))
    return o



def tf_matrix(io, frame):
    """base_link -> frame from TF (4x4), or None."""
    try:
        t = io.tf_buffer.lookup_transform(phs.BASE_FRAME, frame, rclpy.time.Time())
    except Exception:
        return None
    tr, q = t.transform.translation, t.transform.rotation
    return phs.T_from([tr.x, tr.y, tr.z], Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix())


class SerlServer:
    def __init__(self, io, port_base, teach=None, host="*"):
        self.io, self.t = io, teach
        ctx = zmq.Context.instance()
        self.pub = ctx.socket(zmq.PUB)
        self.pub.bind(f"tcp://{host}:{port_base + wire.PORT_OBS}")
        self.sub = ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.sub.bind(f"tcp://{host}:{port_base + wire.PORT_CONTROL}")
        self.episode_id, self.step = 0, 0
        self.state, self.reason, self.reward, self.action = wire.FS_IDLE, wire.TR_NONE, 0.0, np.zeros(6)
        self.counts = {"delta": 0, "env_cmd": 0, "bad": 0, "frames": 0}
        self.last_pub, self.delta, self.delta_t = 0.0, None, 0.0
        self.running, self.paused, self.abort = False, False, False
        self.watch = None                                 # (hole pos, axis) the hole must not leave during a retract
        self.j7_ref = float(io.effort[6]) if io.effort is not None else 0.0   # unloaded left j7 [mA]
        self.j7_since, self.retract_floor = None, None
        o = json.load(open(OFFSET_MODEL))["const"] if OFFSET_MODEL.exists() else [0.0, 0.0]
        self.set_offset(o)
        print(f"peg offset {o[0]:+.2f} {o[1]:+.2f} mm (peg tool y/z, {OFFSET_MODEL.name}): on-axis tests use it")
        self.next_pair = None                             # (hole EE, peg start rel) for the next reset only
        self.expert = None                                # the machine intervention's Expert, this episode
        self.seat = None                                  # (offset, force, depth): seat_hole() in every reset
        self.seat_path = None                             # taught waypoints (peg_hole_waypoint_gui.py), before the press
        # episode starts: EnvCmd RESET params[0] (wire.RESET_*); retries from the last good state
        self.reset_mode, self.auto_reset, self.max_retries = wire.RESET_AUTO, False, 3
        self.good_rel = None                              # last good state: commanded IK EE pose in the hole tool frame
        self.last_end = None                              # (reason, episode_id) of the last episode
        self.reset_kind, self.retry_count, self.parent_episode = 0, 0, 0
        self.cams, self.last_images = None, None          # images.WristCameras (--images): newest crops per tick
        # Frame state: obs limit boxes, EE velocity memory, the episode's priv values
        self.profiles = observation.load_profiles()
        self.limit_T = {}                                 # arm -> its (static) profile frame, once on TF
        self.prev_sites, self.prev_t = None, None
        self.info, self.ep_return, self.T_cmd, self.clamped = {}, 0.0, None, False
        self.raw_delta = None
        print("limit profiles: " + ", ".join(f"{a}: {f}" for a, (f, _, _) in self.profiles.items()))
        if teach is not None:
            self.em = EpisodeMachine()
            inner = teach.st.on_tick
            teach.st.on_tick = lambda: (inner(), self.background())

    def set_offset(self, mm):
        """Peg-tool (y, z) offset [mm] the on-axis rules measure from (the calibrated hole axis)."""
        self.offset_mm = np.array(mm, dtype=float)
        # calibrated hole axis in the hole tool frame: the peg-tool y/z offset rotated by the roll
        self.axis_c = (Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix()
                       @ np.r_[0.0, self.offset_mm] / 1000.0)[1:]

    # --- wire ---------------------------------------------------------------------------
    def poll_commands(self):
        """Drain the control socket: keep the newest delta, act on EnvCmds."""
        while self.sub.poll(0):
            data = self.sub.recv()
            try:
                t = wire.msg_type(data)
                if t == wire.MSG_CONTROL_DELTA:
                    self.delta, self.delta_t = np.array(wire.decode_delta(data)[0][:6]), time.monotonic()
                    self.counts["delta"] += 1
                elif t == wire.MSG_ENV_CMD:
                    self.counts["env_cmd"] += 1
                    self.on_env_cmd(wire.decode_env_cmd(data)[0])
                else:
                    self.counts["bad"] += 1
            except ValueError:
                self.counts["bad"] += 1

    def on_env_cmd(self, c):
        cmd = c["cmd"]
        print(f"EnvCmd {wire.CMD_NAMES.get(cmd, cmd)} episode {c['episode_id']}")
        if self.t is None:
            return
        if cmd == wire.RESET:
            self.running, self.paused = True, False
            m = int(round(c["params"][0])) if c["params"] else wire.RESET_AUTO
            self.reset_mode = m if m in (wire.RESET_AUTO, wire.RESET_RANDOM, wire.RESET_RETRY) else wire.RESET_AUTO
            print(f"  reset mode {wire.RESET_MODE_NAMES[self.reset_mode]}")
            if c["episode_id"]:
                self.episode_id = c["episode_id"] - 1             # the next episode gets this id
        elif cmd == wire.PAUSE:
            self.paused = True
        elif cmd == wire.RESUME:
            self.paused = False
        elif cmd == wire.ABORT:
            self.abort = True

    def publish(self):
        vecs = self.frame_vectors()
        if vecs is None:
            return False
        self.last_frame = wire.encode_frame(*vecs, self.episode_id, self.step, self.state, self.reason,
                                            self.reward, self.action)
        if self.cams is None:
            self.pub.send(self.last_frame)
        else:                                             # [Frame, right RGB, left RGB]: one message per tick
            self.pub.send_multipart(wire.frame_parts(self.last_frame, self.last_images))
        self.counts["frames"] += 1
        self.last_pub = time.monotonic()
        return True

    def frame_vectors(self):
        """(obs, priv) for this tick (wire.OBS_FIELDS / PRIV_FIELDS), or None until the robot
        state is in. Geometry is measured live in every state; episode values come from
        self.info (set by episode_tick / reset, NaN outside an episode)."""
        o = gateway_obs(self.io)
        T_l, T_r = self.io.ee("left"), self.io.ee("right")
        if o is None or T_l is None or T_r is None:
            return None
        sites = {a: self.io.latest_site(a) for a in ("right", "left")}
        now = time.monotonic()
        dt = 0.0 if self.prev_t is None else now - self.prev_t
        vel = {a: observation.velocity(None if self.prev_sites is None else self.prev_sites[a], sites[a], dt)
               for a in sites}
        self.prev_sites, self.prev_t = sites, now
        limits, frame_ok = {}, {}
        for a in ("right", "left"):
            k = a[0]
            if k in self.profiles and k not in self.limit_T:
                T_f = tf_matrix(self.io, self.profiles[k][0])
                if T_f is not None:
                    self.limit_T[k] = T_f
            T_f = self.limit_T.get(k)
            frame_ok[a] = T_f is not None
            limits[a] = (observation.limit_margins(sites[a], T_f, *self.profiles[k][1:]) if T_f is not None
                         else np.full(12, wire.NO_LIMIT_DIFF))
        obs = observation.build_obs(o.joint_pos, o.joint_vel, o.joint_effort,
                                    {a: observation.pose6(sites[a]) for a in sites}, vel, limits)
        p = {name: np.zeros(n) for name, n, _ in wire.PRIV_FIELDS}     # no NaN: n/a = 0 (+ flags)
        p["delta_age"], p["image_age"] = np.array([wire.NO_NAN_CAP]), np.full(2, wire.NO_NAN_CAP)
        p["gateway_obs"] = np.array(wire._OBS.unpack_from(gw.encode_obs(o), wire._HEADER.size))
        H = T_l @ pht.HOLE_TOOL
        P = np.linalg.inv(H) @ T_r @ pht.PEG_TOOL
        lat_yz = np.array([P[1, 3] - self.axis_c[0], P[2, 3] - self.axis_c[1]])
        p.update(hole_pose=observation.pose6(H), peg_in_hole=observation.pose6(P), lateral_yz=lat_yz,
                 lateral=np.linalg.norm(lat_yz), depth=pht.TOP - P[0, 3],
                 tilt=np.degrees(np.arccos(np.clip(P[0, 0], -1.0, 1.0))),
                 hole_offset=self.offset_mm / 1000.0,
                 j7_left_change=abs(float(self.io.effort[6]) - self.j7_ref),
                 limit_frame_ok=np.array([frame_ok["right"], frame_ok["left"]], float),
                 mode={wire.FS_IDLE: 0, wire.FS_RESET: 4, wire.FS_FAULT: 5}.get(self.state, 0))
        if self.cams is not None:
            self.last_images, img_t = self.cams.grab()
            t_now = time.time()
            ok = np.array([self.last_images[c] is not None for c in ("right", "left")], float)
            age = np.array([t_now - img_t[c] if self.last_images[c] is not None else wire.NO_NAN_CAP
                            for c in ("right", "left")])
            p["image_ok"], p["image_age"] = ok, np.clip(age, 0.0, wire.NO_NAN_CAP)
        if self.state in (wire.FS_POLICY, wire.FS_INTERVENTION, wire.FS_TERMINATED):
            p.update(self.info)
        priv = np.concatenate([np.atleast_1d(np.asarray(p[name], float)) for name, _, _ in wire.PRIV_FIELDS])
        for name, vec, fields in (("obs", obs, wire.OBS_FIELDS), ("priv", priv, wire.PRIV_FIELDS)):
            bad = ~np.isfinite(vec)
            if bad.any():                                 # never send NaN / inf (the learner drops them)
                where = sorted({f for f, n, _ in fields for i in range(*(wire.OBS_SLICES if name == "obs"
                                else wire.PRIV_SLICES)[f]) if bad[i]})
                if where != getattr(self, "_nan_warned", {}).get(name):
                    print(f"  WARNING: non-finite {name} fields {where} sent as 0")
                    self.__dict__.setdefault("_nan_warned", {})[name] = where
                vec[bad] = 0.0
        return obs, priv

    def episode_info(self, ins, s_cmd, r=None, blocked=False, safety=False, hole_shift=0.0, dj7=0.0):
        """priv values of an episode tick (r = EpisodeMachine.tick's result; None = step 0)."""
        rw = self.em.reward
        terms = r["terms"] if r is not None else {"success": 0.0, "shape": 0.0, "force": 0.0, "fail": 0.0,
                                                  "rim_strike": False}
        reward = r["reward"] if r is not None else 0.0
        self.ep_return = (self.ep_return if r is not None else 0.0) + reward
        reason = r["reason"] if r is not None else wire.TR_NONE
        fresh = self.raw_delta is not None
        return {
            "reward": reward, "return": self.ep_return,
            "r_success": terms["success"], "r_shape": terms["shape"], "r_force": terms["force"], "r_fail": terms["fail"],
            "push_back": rw.push_back, "push_back_peak": rw.f_peak,
            "force_axial": 0.0 if not np.isfinite(rw.f_axial) else rw.f_axial,
            "force_ref": 0.0 if rw.ref is None else rw.ref, "force_ref_valid": float(rw.ref is not None),
            "depth_cmd": pht.TOP - s_cmd,
            "ee_right_cmd": observation.pose6(self.T_cmd),
            "policy_delta": self.raw_delta if fresh else np.zeros(6), "policy_delta_valid": float(fresh),
            "delta_age": min(time.monotonic() - self.delta_t, wire.NO_NAN_CAP) if self.delta_t else wire.NO_NAN_CAP,
            "clamped": float(self.clamped),
            "mode": wire.MODE_CODES[self.em.mode],
            "interventions": self.em.interventions, "t_episode": self.em.t,
            "reset_kind": float(self.reset_kind), "retry_count": float(self.retry_count),
            "parent_episode": float(self.parent_episode),
            "can_touch": float(self.em.can_touch(ins)), "in_zone": float(ins["lateral"] <= ZONE),
            "success": float(reason == wire.TR_SUCCESS),
            "failed": float(reason in (wire.TR_JAM, wire.TR_RIM)),
            "rim_strike": float(bool(terms["rim_strike"])),
            "blocked": float(blocked), "safety": float(safety),
            "hole_shift": hole_shift, "j7_left_change": dj7,
        }

    def background(self):
        """Every 100 Hz control tick: keep RESET / IDLE / FAULT frames flowing at 15 Hz while
        the robot runs a blocking motion (the reset), keep reading commands, and stop a
        retract that pushes the hole."""
        if self.watch is not None and self.sideways(self.watch) > HOLE_SHIFT_MAX:
            self.watch = None
            raise HoleMoved(f"hole pushed {HOLE_SHIFT_MAX * 1000:.0f}+ mm sideways")
        if self.t is not None and self.state in (wire.FS_POLICY, wire.FS_INTERVENTION):
            self.guard()                                      # episodes only: resets press on purpose (user 2026-10-05)
        if self.state in (wire.FS_RESET, wire.FS_IDLE, wire.FS_FAULT) and time.monotonic() - self.last_pub >= 1.0 / HZ:
            self.poll_commands()
            self.publish()

    def guard(self):
        """Gear guard on the left j7 current (see J7_*)."""
        d = abs(float(self.io.effort[6]) - self.j7_ref)
        if self.retract_floor is not None:                     # retract: only a rising load trips
            if d > max(J7_HARD, self.retract_floor + J7_RISE):
                raise GuardTrip(f"left j7 load rose to {d:.0f} mA while retracting")
            return
        if d > J7_HARD:
            raise GuardTrip(f"left j7 +{d:.0f} mA > {J7_HARD:.0f}")
        if d > J7_SUSTAIN:
            self.j7_since = self.j7_since if self.j7_since is not None else time.monotonic()
            if time.monotonic() - self.j7_since > J7_HOLD:
                raise GuardTrip(f"left j7 > {J7_SUSTAIN:.0f} mA for {J7_HOLD:.2f} s")
        else:
            self.j7_since = None

    # --- measurements ---------------------------------------------------------------------
    def hole_now(self):
        """(hole tool point, insertion axis) in base_link, from TF."""
        H = self.io.ee("left") @ pht.HOLE_TOOL
        return H[:3, 3].copy(), H[:3, 0].copy()

    def sideways(self, ref):
        """How far the hole has moved across its axis since `ref` = (point, axis) [m]."""
        d = self.hole_now()[0] - ref[0]
        return float(np.linalg.norm(d - (d @ ref[1]) * ref[1]))

    def peg_in_hole(self, T_peg_ee):
        """(height on the axis, lateral from the CALIBRATED axis, tilt deg) of a right EE pose."""
        P = np.linalg.inv(self.io.ee("left") @ pht.HOLE_TOOL) @ T_peg_ee @ pht.PEG_TOOL
        lat = float(np.hypot(P[1, 3] - self.axis_c[0], P[2, 3] - self.axis_c[1]))
        return float(P[0, 3]), lat, float(np.degrees(np.arccos(np.clip(P[0, 0], -1.0, 1.0))))

    def measure(self):
        """(ins, q, amps, lift, s_real, s_cmd) of the right arm / peg now (lateral from the
        calibrated axis)."""
        s_real, lat, tilt = self.peg_in_hole(self.io.ee("right"))
        arm = self.t.st.arms["right"]
        s_cmd = self.peg_in_hole(arm.last_goal_eff)[0] if arm.last_goal_eff is not None else s_real
        raw = self.io.raw_js
        idx = {n: i for i, n in enumerate(raw[1])}
        q = np.array([raw[2][idx[n]] for n in RIGHT])
        amps = np.array([raw[4][idx[n]] for n in RIGHT]) * effort.EFF_TO_A
        ins = {"depth": pht.TOP - s_real, "lateral": lat, "across": (lat, 0.0), "tilt_deg": tilt}
        if not self.em.can_touch(ins):                        # contact impossible: re-zero the hole watch
            self.hole_ref, self.j7_ref = self.hole_now(), float(self.io.effort[6])   # (and the gear guard)
        return ins, q, amps, raw[2][idx["lift_joint"]], s_real, s_cmd

    def site_cmd(self):
        """The right arm's commanded IK EE pose (what Obs.ee and the deltas refer to)."""
        arm = self.t.st.arms["right"]
        T_tf = arm.goal if arm.anchor is None else self.io.ee("left") @ np.linalg.inv(arm.anchor) @ arm.goal
        return T_tf @ arm.map

    def command_site(self, T_site, policy_limits=True):
        """Send a commanded IK EE pose (base-fixed goal), clamped to the policy's box around the hole and
        its above-rim floor outside the zone. policy_limits=False: the reset's own scripted moves (the
        seat presses) -- those limits are for the policy; the gear guard and the hole watch still run."""
        arm = self.t.st.arms["right"]
        if not policy_limits:
            arm.anchor, arm.goal = None, T_site @ np.linalg.inv(arm.map)
            return arm.goal @ arm.map
        T_peg = T_site @ np.linalg.inv(arm.map) @ pht.PEG_TOOL
        T_l = self.io.ee("left")
        H = T_l @ pht.HOLE_TOOL
        R_ref = (H @ phs.T_from([0, 0, 0], Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix()))[:3, :3]
        box = SafetyBox(H, [pht.TOP - self.t.a.push - 0.003, -BOX_ACROSS, -BOX_ACROSS],
                        [pht.TOP + BOX_UP, BOX_ACROSS, BOX_ACROSS], [BOX_TILT] * 3, R_ref)
        T_peg, hit = box.clamp(T_peg)
        self.clamped = bool(np.any(hit))
        p = np.linalg.inv(H) @ np.r_[T_peg[:3, 3], 1.0]                   # x = height on the axis
        if np.hypot(p[1] - self.axis_c[0], p[2] - self.axis_c[1]) > ZONE and p[0] < pht.TOP + FLOOR:
            # outside the interaction zone: stay above the rim
            self.clamped = True
            p[0] = pht.TOP + FLOOR
            T_peg = T_peg.copy()
            T_peg[:3, 3] = (H @ p)[:3]
        arm.anchor, arm.goal = None, T_peg @ np.linalg.inv(pht.PEG_TOOL)
        return arm.goal @ arm.map

    # --- episodes -------------------------------------------------------------------------
    def reset(self):
        """RESET frames while the teach tool resets; -> True when the next episode starts."""
        self.state, self.reason, self.reward, self.action = wire.FS_RESET, wire.TR_NONE, 0.0, np.zeros(6)
        self.info, self.T_cmd, self.expert = {}, None, None
        retry = self.retry_wanted()
        try:
            self.retract()
            self.j7_ref, self.j7_since = float(self.io.effort[6]), None        # peg clear: unloaded reference
            if retry:
                ok = self.retry_start()
            elif self.seat is None:
                ok = self.t.hw_reset(self.start_pair())
            else:
                # the hole moves with the peg riding above it (no trip to the peg start and back): seat
                # the hole, THEN go to the start once
                go_start, self.t.go_start = self.t.go_start, lambda tries=10: True
                try:
                    ok = self.t.hw_reset(self.start_pair())
                finally:
                    self.t.go_start = go_start
                ok = ok and self.seat_hole() and self.t.go_start()
        except (phr.LagTrip, HoleMoved, GuardTrip) as e:
            print(f"reset stopped: {e}")
            ok = False
        if not ok:
            self.watch, self.state, self.running = None, wire.FS_FAULT, False
            print("reset failed: FAULT, holding (check the robot, then EnvCmd RESET)")
            return False
        if self.paused:                                       # PAUSE arrived during the reset
            self.state = wire.FS_IDLE
            return False
        self.t.take_right()
        if retry:
            self.reset_kind, self.retry_count, self.parent_episode = 1, self.retry_count + 1, self.last_end[1]
        else:
            self.reset_kind, self.retry_count, self.parent_episode = 0, 0, 0
        self.episode_id += 1
        self.step = 0
        self.T_cmd = self.command_site(self.site_cmd())
        ins, q, amps, lift, s_real, s_cmd = self.measure()
        self.em.start(self.episode_id, ins, q, amps, lift, self.T_cmd)
        self.block = pht.BlockDetector(rim_s=pht.TOP_RECORDED)
        self.edge = pht.EdgeForce(pht.TOP, self.t.a.edge_force, hard=self.t.a.hard_edge)
        self.e0 = self.io.effort.copy()
        self.hole_ref, self.j7_ref = self.hole_now(), float(self.io.effort[6])
        self.t0 = time.monotonic()
        self.state, self.action = wire.FS_POLICY, np.zeros(6)
        self.raw_delta = None
        self.good_rel = np.linalg.inv(self.io.ee("left") @ pht.HOLE_TOOL) @ self.T_cmd   # step 0 is always clear
        self.info = self.episode_info(ins, s_cmd)
        self.publish()                                        # step 0: the observation after the reset
        print(f"episode {self.episode_id}: start ({'retry %d of episode %d' % (self.retry_count, self.parent_episode) if retry else 'new pair'}), "
              f"peg depth {ins['depth'] * 1000:+.1f} mm")
        return True

    def retry_wanted(self):
        """Does this reset retry the last episode's pair from its good state? (wire.RESET_* mode)"""
        if self.next_pair is not None or self.last_end is None or self.good_rel is None:
            return False                                  # an explicit pair (recorder), or nothing to retry
        reason = self.last_end[0]
        if reason in (wire.TR_SAFETY, wire.TR_ABORT):     # hole pose not trusted / stopped by hand
            return False
        if self.reset_mode == wire.RESET_RETRY:
            return True
        return (self.reset_mode == wire.RESET_AUTO and reason in wire.TR_RETRY
                and self.retry_count < self.max_retries)

    def retry_start(self):
        """Same hole (re-seated: the failure may have unseated it), peg back to the last good state:
        the last policy pose with the tip >= GOOD_CLEAR above the rim, kept relative to the hole."""
        t = self.t
        if self.seat is not None and not self.seat_hole():
            return False
        arm = t.st.arms["right"]
        H = self.io.ee("left") @ pht.HOLE_TOOL
        T_tf = H @ self.good_rel @ np.linalg.inv(arm.map)              # IK site goal -> TF EE goal
        above = T_tf.copy()
        above[:3, 3] += 0.01 * H[:3, 0]
        self.quick_move(above, t.a.move_speed, "hover")               # both ends above the rim: the line is too
        t.move_right(T_tf, t.a.speed, "to_top")
        s_real = self.peg_in_hole(self.io.ee("right"))[0]
        if pht.TOP - s_real > -GOOD_CLEAR:                            # not clear: the episode must start where
            up = s_real - pht.TOP                                     # the force reference can be taken
            T_up = T_tf.copy()
            T_up[:3, 3] += (GOOD_CLEAR + 0.001 - up) * H[:3, 0]
            print(f"  retry start {up * 1000:+.1f} mm from the rim: lifted to {(GOOD_CLEAR + 0.001) * 1000:.0f} mm above")
            t.move_right(T_up, t.a.speed, "to_top")
        t.phase("idle")
        return True

    def after_episode(self):
        """After TERMINATED: unless --auto-reset, pull the peg straight out NOW (it may be loaded against
        the hole), then hold IDLE until the client's next EnvCmd RESET."""
        self.last_end = (self.reason, self.episode_id)
        if self.auto_reset or self.paused:
            return
        self.state = wire.FS_RESET
        try:
            self.retract()
            self.state = wire.FS_IDLE
        except (phr.LagTrip, HoleMoved, GuardTrip) as e:
            print(f"pull-out after the episode stopped: {e}: FAULT")
            self.watch, self.state = None, wire.FS_FAULT
        self.running = False

    def seat_hole(self):
        """Seat the hole block in the left gripper before the episode: (optionally a taught path first, then)
        press the peg down on the block beside the hole to self.seat's force, lift it back up.
        2026-10-04 demos: the first contact after a reset moved the hole another 0.7-0.8 mm (retries
        0.05-0.2) -- most first tries failed, their retries went in. RESET frames; a scripted move:
        no policy limits, but the gear guard and the hole watch (> 6 mm sideways -> FAULT) stay on."""
        off_y, f_max, _ = self.seat
        if self.seat_path is not None:
            self.run_path(self.seat_path)
        if f_max is None:                                         # --no-seat-press: the taught path only
            self.t.phase("idle")
            return True
        OFF = phs.T_from([0.0, self.offset_mm[0] / 1000.0 + off_y, self.offset_mm[1] / 1000.0], np.eye(3))
        r = self.press(OFF, f_max)
        h0 = r["hole0"]
        self.t.held(0.15)
        moved = self.sideways(h0)
        self.lift_off(OFF, r["s_cmd"])
        print(f"  seat: {self.press_line(r)}, {off_y * 1000:.0f} mm beside the hole; hole moved {moved * 1000:.2f} mm")
        self.t.phase("idle")
        return True

    def press(self, OFF, f_target, past_contact=0.006, start=0.005):
        """Force-controlled press with the peg at peg-tool offset OFF: straight to `start` above the
        modelled block top, wait for a still hole, then down at half --speed until the push-back passes
        f_target (after contact) -- or `past_contact` past where contact began (push-back > 3 N), or
        12 mm below the modelled top with no contact at all (pressing into air). By force, not by
        the modelled top: live 2026-10-05 the block sat 3.3 mm lower and a 4 mm cap pressed ~1 mm.
        -> dict(s_cmd, f, d_contact, d, hole0)."""
        t, arm = self.t, self.t.st.arms["right"]
        self.quick_move(t.peg_target(pht.TOP + start, OFF)[0], t.a.move_speed, "hover")
        self.wait_hole_still()
        h0 = self.hole_now()
        rw = self.em.reward
        rw.reset()
        s_cmd, f, s_contact, d_contact, k, over = pht.TOP + start, 0.0, None, None, 0, 0
        t.phase("push")
        while True:
            k += 1
            s_cmd -= 0.5 * t.a.speed / HZ                       # constant: a speed change fakes force (friction)
            self.command_site(t.peg_target(s_cmd, OFF)[0] @ arm.map, policy_limits=False)
            t_end = time.monotonic() + 1.0 / HZ
            while time.monotonic() < t_end - phs.CTRL_DT / 2:
                t.st.tick(None)
            ins, q, amps, lift, _, _ = self.measure()
            f = rw.push_back_force(ins, q, amps, lift)
            over = over + 1 if f > 3.0 and k > 5 else 0         # the first ticks' start transient is not contact
            if s_contact is None and over >= 2:
                s_contact, d_contact = s_cmd, ins["depth"]
            elif s_contact is not None and f > f_target:
                break
            if (s_contact is not None and s_cmd < s_contact - past_contact) or s_cmd < pht.TOP - 0.012:
                break
        return {"s_cmd": s_cmd, "f": f, "d_contact": d_contact, "d": ins["depth"], "hole0": h0}

    @staticmethod
    def press_line(r):
        c = "no contact" if r["d_contact"] is None else f"contact at {r['d_contact'] * 1000:+.1f} mm"
        return f"pressed {r['f']:.1f} N, {c}, peg {r['d'] * 1000:+.1f} mm under the modelled top"

    def lift_off(self, OFF, s_cmd):
        """5 mm straight up off the block (go_start() follows)."""
        self.quick_move(self.t.peg_target(max(s_cmd, pht.TOP) + 0.005, OFF)[0], self.t.a.move_speed, "retract")

    def quick_move(self, T, speed, phase):
        """teach.move_right with a QUICK_SETTLE pause (the lag check still runs)."""
        t = self.t
        t.phase(phase)
        t.st.move("right", T, speed, None, anchor=self.io.ee("left"))
        t.st.settle(QUICK_SETTLE, t.a.reach_lag, t.a.settle_timeout)

    def hole_lateral(self, y_h, z_h):
        """Peg-tool offset that puts the peg tool point at (y_h, z_h) [m] across the CALIBRATED hole
        axis, in the hole tool frame (peg_target's offset is in the ROLL-rotated peg frame)."""
        R = Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix()
        p = R.T @ np.r_[0.0, self.axis_c[0] + y_h, self.axis_c[1] + z_h]
        return phs.T_from([0.0, p[1], p[2]], np.eye(3))

    def wait_hole_still(self, timeout=1.0, v_still=0.0005, window=0.15):
        """Up to `timeout` s until the hole moves < v_still [m/s] over `window` s (it settles after a move)."""
        hist, t_end = [], time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.t.st.tick(None)
            hist = (hist + [(time.monotonic(), self.hole_now()[0])])[-30:]
            while len(hist) > 2 and hist[-1][0] - hist[1][0] >= window:
                hist.pop(0)
            if hist[-1][0] - hist[0][0] >= window and \
                    np.linalg.norm(hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0]) < v_still:
                return

    def run_path(self, wps, v_press=0.015, v_exit=0.03):
        """Replay taught waypoints (peg_hole_waypoint_gui.py) at the LIVE hole: each is the commanded peg
        tool pose relative to the hole tool frame (full pose: a tilted peg replays tilted). "move"
        waypoints at --move-speed, "press" ones -- and any segment that starts or ends below the block
        top + 3 mm -- at v_press. Getting to the first waypoint: 10 mm above it along the hole axis
        first, then down slowly; leaving the hole for a waypoint above the top: straight up the axis
        (same pose) first. A scripted reset move: no policy limits; the gear guard and the hole watch
        stay on. Ends 5 mm up the axis."""
        t = self.t
        H = self.io.ee("left") @ pht.HOLE_TOOL
        h0, ax = self.hole_now()
        Hi = np.linalg.inv(H)

        def low(T):                                               # peg tool near / under the block top
            return (Hi @ T @ pht.PEG_TOOL)[0, 3] < pht.TOP + 0.003

        def up_axis(T, d):
            U = T.copy()
            U[:3, 3] += d * H[:3, 0]
            return U

        prev = None
        for w in wps:
            T = H @ np.array(w["rel_cmd"]) @ np.linalg.inv(pht.PEG_TOOL)       # TF EE goal
            slow = w["kind"] == "press" or low(T) or (prev is not None and low(prev))
            if prev is None:
                self.quick_move(up_axis(T, 0.010), t.a.move_speed, "hover")      # 10 mm above the first, same pose
                self.wait_hole_still()
            elif low(prev) and not low(T):                                  # leaving the hole: straight up first
                rise = pht.TOP + 0.005 - (Hi @ prev @ pht.PEG_TOOL)[0, 3]
                self.quick_move(up_axis(prev, rise), v_exit, "retract")
                slow = False
            self.quick_move(T, v_press if slow else t.a.move_speed, "push" if slow else "hover")
            prev = T
        dh = self.hole_now()[0] - h0
        side = dh - (dh @ ax) * ax
        self.quick_move(up_axis(self.io.ee("right"), 0.005), t.a.move_speed, "retract")
        lat = side @ H[:3, :3]                                   # hole frame components of the sideways move
        print(f"  path: {len(wps)} waypoints ({sum(w['kind'] == 'press' for w in wps)} press); block moved "
              f"{np.linalg.norm(side) * 1000:.2f} mm sideways (base x {side[0] * 1000:+.2f}, y {side[1] * 1000:+.2f} mm; "
              f"hole frame y {lat[1] * 1000:+.2f}, z {lat[2] * 1000:+.2f})")
        t.phase("idle")

    def peg_lateral(self):
        """Measured peg tool point across the hole axis, hole tool frame (y, z) [m]."""
        P = np.linalg.inv(self.io.ee("left") @ pht.HOLE_TOOL) @ self.io.ee("right") @ pht.PEG_TOOL
        return P[1:3, 3].copy()

    def start_pair(self):
        """None = the teach tool's random reset (peg start +- --xy across, tilted). With
        --start-xy (tests): a random hole, the peg 2 cm above the rim on the CALIBRATED axis
        +- start_xy across it, aligned -- straight down then goes in."""
        if self.next_pair is not None:
            pair, self.next_pair = self.next_pair, None
            return pair
        xy = getattr(self.t.a, "start_xy", None)
        if xy is None:
            return None
        rng = self.t.rng
        for _ in range(20):
            hole_ee, _ = self.t.sample()
            roll = Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix()
            off = self.offset_mm / 1000.0 + rng.uniform(-xy, xy, 2)
            peg_rel = (pht.HOLE_TOOL @ phs.T_from([pht.TOP + 0.02, 0, 0], roll) @ phs.T_from([0.0, *off], np.eye(3))
                       @ np.linalg.inv(pht.PEG_TOOL))
            if self.t.check_sample(hole_ee, peg_rel)[0]:
                return hole_ee, peg_rel
        return None

    def retract(self):
        """Peg straight up the hole axis (keeping its offset) until its tool point is CLEAR
        above ON_TOP, with the hole watched: never sideways while it can touch the block."""
        self.t.take_right()
        T_l = self.io.ee("left")
        H = T_l @ pht.HOLE_TOOL
        T_peg = self.io.ee("right") @ pht.PEG_TOOL
        rise = pht.TOP + self.t.a.clear - (np.linalg.inv(H) @ np.r_[T_peg[:3, 3], 1.0])[0]
        if rise <= 0.0:
            return
        # in the bore at RETRACT_SPEED, then (tip 2 mm clear) the rest at the free-move speed: the
        # whole 54 mm at --speed took 5.9 s of every reset
        in_bore = min(rise, max(0.0, rise - self.t.a.clear + 0.002))
        legs = [(in_bore, RETRACT_SPEED), (rise, self.t.a.move_speed)] if in_bore > 0.0 else [(rise, self.t.a.move_speed)]
        self.watch = self.hole_now()
        self.retract_floor = abs(float(self.io.effort[6]) - self.j7_ref)
        try:
            for k, (up, speed) in enumerate(legs):
                T_up = T_peg.copy()
                T_up[:3, 3] += up * H[:3, 0]
                (self.quick_move if k < len(legs) - 1 else self.t.move_right)(
                    T_up @ np.linalg.inv(pht.PEG_TOOL), speed, "reset_out")
        finally:
            self.watch, self.retract_floor = None, None

    def trace_target(self):
        """Where the machine's trace-back goes: the last policy pose near the hole top, or (none
        yet) the peg aligned on the calibrated axis just above the rim."""
        if self.em.restore is not None:
            return self.em.restore
        off = phs.T_from([0.0, *(self.offset_mm / 1000.0)], np.eye(3))
        return self.t.peg_target(pht.TOP + self.em.cfg.clear, off)[0] @ self.t.st.arms["right"].map

    def episode_tick(self):
        """One 15 Hz frame of an episode: act (policy or machine), let the robot move for the
        rest of the period, measure, tag, publish. -> False once TERMINATED."""
        period, t_end = 1.0 / HZ, time.monotonic() + 1.0 / HZ
        mode = self.em.mode
        early_trip = None
        if mode == "policy" and self.delta is None:
            # lockstep: the client answers frame k right after it is published, i.e. while this tick
            # starts -- wait (arms streaming) for that answer so it drives THIS tick and frame k+1's
            # tag.action is the action chosen from frame k. Without it every action landed a tick late.
            t_wait = time.monotonic() + POLICY_WAIT
            try:
                while self.delta is None and time.monotonic() < t_wait:
                    self.poll_commands()
                    if self.delta is None:
                        self.t.st.tick(None)
            except (GuardTrip, HoleMoved) as e:
                early_trip = e
        if mode == "policy":
            stale = self.delta is None or time.monotonic() - self.delta_t > WATCHDOG_S
            d = np.zeros(6) if stale else clip_delta(self.delta, MAX_TRANS, MAX_ROT)
            self.raw_delta = None if stale else self.delta.copy()
            self.delta = None
        elif mode == "pull_out":
            d = self.em.pull_out_delta((self.io.ee("left") @ pht.HOLE_TOOL)[:3, 0])
            self.raw_delta = None
        elif mode == "expert":                                 # SERL machine intervention: the expert to the goal
            if self.expert is None:
                self.expert = Expert(self, EXPERT_CFG, self.offset_mm)
                print(f"  step {self.step}: the expert takes over")
            d = self.expert.delta()
            self.raw_delta = None
        else:                                                 # trace_back
            target = self.trace_target()
            d = self.em.toward_delta(self.T_cmd, target)
            self.raw_delta = None
        self.T_cmd = self.command_site(apply_delta(self.T_cmd, d))
        self.action = d
        tripped = None if early_trip is None else str(early_trip)
        try:
            while tripped is None and time.monotonic() < t_end - phs.CTRL_DT / 2:
                self.t.st.tick(None)
        except (GuardTrip, HoleMoved) as e:                   # gear guard: end the episode at once
            tripped = str(e)
            print(f"  step {self.step + 1}: SAFETY stop: {e}")
        ins, q, amps, lift, s_real, s_cmd = self.measure()
        if mode == "policy" and ins["depth"] <= -GOOD_CLEAR and pht.TOP - s_cmd <= -GOOD_CLEAR:
            # the last good state: outside the hole -- the measured AND the commanded tip (the stored
            # pose is the command, ~3 ticks ahead of the arm: live, measured-only gave starts 1.4 mm in)
            self.good_rel = np.linalg.inv(self.io.ee("left") @ pht.HOLE_TOOL) @ self.T_cmd
        now = time.monotonic() - self.t0
        blocked = self.block.update(now, s_real, s_cmd) is not None
        safety = tripped is not None or self.edge.update(now, s_real, self.io.raw_js) == "edge_hard" or \
            abs(self.io.effort[6] - self.e0[6]) > self.t.a.hard_j7 or self.abort
        arrived = mode == "trace_back" and \
            all(e < tol for e, tol in zip(pose_error(self.T_cmd, self.trace_target()), (0.001, np.radians(1.0))))
        hole_shift, dj7 = self.sideways(self.hole_ref), abs(float(self.io.effort[6]) - self.j7_ref)
        r = self.em.tick(period, ins, q, amps, lift, self.T_cmd, blocked=blocked, safety=safety, arrived=arrived,
                         hole_shift=hole_shift, dj7=dj7)
        if self.abort and r["frame_state"] == wire.FS_TERMINATED:
            r["reason"], self.abort, self.paused = wire.TR_ABORT, False, True
        self.step = r["step"]
        self.state, self.reason, self.reward = r["frame_state"], r["reason"], r["reward"]
        self.info = self.episode_info(ins, s_cmd, r, blocked, safety, hole_shift, dj7)
        self.publish()
        if r["mode"] != mode and r["mode"] != "reset":
            print(f"  step {self.step}: {mode} -> {r['mode']} (push-back {r['terms']['push_back_N']:.1f} N, "
                  f"depth {ins['depth'] * 1000:.1f} mm)")
        if r["frame_state"] == wire.FS_TERMINATED:
            print(f"episode {self.episode_id}: {wire.TR_NAMES[r['reason']]} at step {self.step}, depth "
                  f"{ins['depth'] * 1000:.1f} mm, {r['interventions']} interventions, reward {r['reward']:+.3f}")
            return False
        return True

    def run(self, duration=None):
        period, t_next, t0, last_log = 1.0 / HZ, time.monotonic(), time.monotonic(), 0.0
        while duration is None or time.monotonic() - t0 < duration:
            self.poll_commands()
            if self.t is not None and self.running and not self.paused:
                if self.reset() and not self.paused:           # PAUSE during the reset: stay IDLE
                    while self.episode_tick():
                        self.poll_commands()
                    self.after_episode()
                    self.state = wire.FS_IDLE if self.paused else self.state
                    self.t.take_right()
                t_next = time.monotonic()
                continue
            self.state = wire.FS_FAULT if self.state == wire.FS_FAULT else wire.FS_IDLE
            if self.t is not None:
                self.t.st.tick(None)                              # hold the arms, keep the hook running
            else:
                self.publish()
            now = time.monotonic()
            if now - last_log > 5.0:
                print(f"  {self.counts}")
                last_log = now
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()) if self.t is None else 0.0)
            if time.monotonic() - t_next > period:                # fell behind: don't burst
                t_next = time.monotonic()


def add_episode_args(ap):
    ap.add_argument("--failure-mode", choices=("terminate", "intervene"), default="terminate",
                    help="terminate: a binding / off-axis / blocked trigger ends the episode (fail penalty) and the "
                         "next RESET retries from the last good state; intervene: machine pull-out, same episode")
    ap.add_argument("--auto-reset", action="store_true",
                    help="reset by itself after every episode (default: pull out, then wait for EnvCmd RESET)")
    ap.add_argument("--max-retries", type=int, default=3, help="RESET AUTO retries of one pair before a new one")


def add_seat_args(ap):
    ap.add_argument("--no-seat", action="store_true", help="skip seating the hole block in each reset")
    ap.add_argument("--seat-offset", type=float, default=0.012,
                    help="m beside the hole axis (peg tool +y) where the reset presses on the block top")
    ap.add_argument("--seat-force", type=float, default=8.0, help="N push-back that ends the press")
    ap.add_argument("--seat-depth", type=float, default=0.004, help="m past the block top the press goes at most")
    ap.add_argument("--no-seat-press", action="store_true", help="skip the 8 N down-press (the --seat-path only)")
    ap.add_argument("--seat-path", default=None,
                    help="waypoints JSON from peg_hole_waypoint_gui.py, replayed at the live hole before the down-press")


def seat_from_args(a):
    if a.no_seat:
        return None
    return (a.seat_offset, None if getattr(a, "no_seat_press", False) else a.seat_force, a.seat_depth)


def path_from_args(a):
    if not getattr(a, "seat_path", None):
        return None
    wps = json.loads(Path(a.seat_path).read_text())["waypoints"]
    print(f"seat path: {len(wps)} waypoints from {a.seat_path}")
    return wps


def main():
    ap = pht.build_parser(__doc__, default_out=str(HERE / "recordings" / "peg_hole_serl"))
    add_seat_args(ap)
    add_episode_args(ap)
    ap.add_argument("--port-base", type=int, default=7601)
    ap.add_argument("--duration", type=float, default=None, help="s, then quit (tests)")
    ap.add_argument("--images", action="store_true", help="decode the wrist D405s (priv image_t; ROI crops)")
    ap.add_argument("--start-xy", type=float, default=None,
                    help="m (tests): start the peg aligned 2 cm above the rim within +- this of the calibrated axis")
    a = ap.parse_args()
    pht.apply_setup(a)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    io = pht.TeachIo(a.allow_motion)
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    t, lift_before, srv = None, None, None
    try:
        t0 = time.monotonic()
        while (gateway_obs(io) is None or io.effort is None or io.ee("left") is None or io.ee("right") is None) \
                and time.monotonic() - t0 < 15.0:
            time.sleep(0.1)
        if gateway_obs(io) is None:
            print("no /joint_states or achieved EE poses in 15 s - is the teleop running?")
            return
        if a.allow_motion:
            lift_now = dict(zip(io.raw_js[1], io.raw_js[2])).get("lift_joint")
            if lift_now is None or abs(lift_now - a.lift) > 0.003:
                print(f"lift is {lift_now}, not {a.lift:+.3f}: move it first (peg_hole_teach.py does); quitting")
                return
            if a.lock_lift:
                lift_before = io.set_joint_locked("lift_joint", True)
            t = pht.Teach(io, a)
            t.set_tool_model(a.tool_model)
            t.st.arms["left"] = phr.Arm("left", io.ee("left"), io.latest_site("left"))
            io.set_hold_side("left", True)
            t.take_right()
            print(f"recording to {t.out}")
        srv = SerlServer(io, a.port_base, t)
        srv.seat, srv.seat_path = seat_from_args(a), path_from_args(a)
        srv.auto_reset, srv.max_retries = a.auto_reset, a.max_retries
        if t is not None:
            srv.em.cfg.failure_mode = a.failure_mode
        if a.images:
            srv.cams = images.WristCameras()
        srv.running = bool(a.autostart and t is not None)
        print(f"serving at {HZ:.0f} Hz on {a.port_base} (+{wire.PORT_OBS} frames, +{wire.PORT_CONTROL} control); "
              + ("waiting for EnvCmd RESET" if t is not None and not srv.running else
                 "episodes start now" if srv.running else "no motion: IDLE frames only"))
        srv.run(a.duration)
        print(f"done: {srv.counts}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if srv is not None and srv.cams is not None:
            srv.cams.stop()                               # decoder threads killed at exit abort the process
        if t is not None:
            print(f"session saved: {t.rec.save_session()}")
            t.summary.close()
        if a.allow_motion:
            io.set_hold(False)
            print("both arms released to the SpaceMouse")
            if a.lock_lift and lift_before is False:
                io.set_joint_locked("lift_joint", False)
        time.sleep(0.2)
        ex.shutdown()
        spin.join(timeout=2.0)
        io.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
