#!/usr/bin/env python3
"""HIL-SERL peg-in-hole server for the REAL robot (ffw_peg_hole_env/SERL_REAL_PLAN.md).

Publishes one Frame per 15 Hz tick on base+0: the gateway's Obs (joint block in A,
achieved IK EE poses, grippers; marker block at defaults) plus the tag
(episode_id, step, frame_state, reason, reward, applied action), and listens on
base+1 for ControlCmdDelta / EnvCmd (ffw_peg_hole_env/wire.py).

Without --allow-motion: IDLE frames only, subscribe-only (a wire check).
With --allow-motion (slices 3-4): episodes, all automatic after the first EnvCmd RESET
(or --autostart):
  RESET frames     peg straight up the hole axis to CLEAR first (watching the hole), then
                   Teach.hw_reset: new random reach-checked pose, move there; a lag trip
                   or a pushed hole -> FAULT at once (no retries)
  POLICY frames    each ControlCmdDelta (right arm, base_link, applied to the commanded
                   IK EE pose = the pose Obs.ee reports, clipped 6.67 mm / 2 deg per
                   axis, clamped to a box around the hole and to "below the rim only on the
                   axis"); no delta for 0.2 s = hold
  INTERVENTION     the machine: pull the peg out along the hole axis, trace back to the
                   last policy pose near the hole top, then the policy again
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
from pathlib import Path

import numpy as np
import rclpy
import rclpy.signals
from rclpy.executors import SingleThreadedExecutor
import zmq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "ffw_peg_hole_env"))
sys.path.insert(0, str(HERE.parents[1] / "ffw_zmqinterface"))
from ffw_peg_hole_env import effort, wire  # noqa: E402
from ffw_peg_hole_env.episode import EpisodeMachine  # noqa: E402
from ffw_peg_hole_env.geometry import SafetyBox, apply_delta, clip_delta, pose_error  # noqa: E402
from ffw_zmqinterface import gateway_node as gwn, protocol as gw  # noqa: E402
import peg_hole_random as phr  # noqa: E402
import peg_hole_stroke as phs  # noqa: E402
import peg_hole_teach as pht  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

HZ = 15.0
RIGHT = [f"arm_r_joint{i}" for i in range(1, 8)]
MAX_TRANS, MAX_ROT = 0.01 * 2 / 3, np.radians(2.0)     # per tick, per axis: the policy's +-1
WATCHDOG_S = 0.2
# workspace box for the peg tool point, hole tool frame (x = insertion axis up, from the
# hole tool origin): along x TOP - push - 3 mm .. TOP + 6 cm, across +-6 cm; tilt +-10 deg
BOX_ACROSS, BOX_UP, BOX_TILT = 0.06, 0.06, np.radians(10.0)
# below the rim only on the axis: farther than ON_AXIS across it, the peg tool point
# stays FLOOR above ON_TOP (2026-10-04: a peg that went down beside the block scraped it,
# and the reset then dragged it sideways into the block, pushing the hole 88 mm)
ON_AXIS, FLOOR = 0.003, 0.002
HOLE_SHIFT_MAX = 0.006                                  # m: hole pushed sideways -> stop (resets too)
# Gear guard, every control tick in every phase (episode, intervention, retract, reset):
# left j7 current change from its unloaded reference > J7_HARD at once, or > J7_SUSTAIN for
# J7_HOLD s. Recordings: clean pushes peak <= 400 mA and never stay > 300 mA longer than
# 0.30 s; the 2026-10-04 reset drag pinned j7 at 1.5 A for ~40 s (this trips at 450 mA,
# ~0.05 s into the drag). During a retract only a RISE of the load trips (unloading is fine).
J7_HARD, J7_SUSTAIN, J7_HOLD, J7_RISE = 450.0, 300.0, 0.35, 150.0
OFFSET_MODEL = HERE.parent / "config" / "peg_hole_offset_model.json"


class HoleMoved(Exception):
    pass


class GuardTrip(Exception):
    pass


def obs_frame(io):
    """Gateway-identical Obs frame from /joint_states + /ik_solver/achieved_ee_pose_{r,l},
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
    return gw.encode_obs(o)


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
        if teach is not None:
            o = json.load(open(OFFSET_MODEL))["const"] if OFFSET_MODEL.exists() else [0.0, 0.0]
            self.offset_mm = np.array(o, dtype=float)
            # calibrated hole axis in the hole tool frame: the peg-tool y/z offset rotated by the roll
            self.axis_c = (Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix() @ np.r_[0.0, o[0], o[1]] / 1000.0)[1:]
            print(f"peg offset {o[0]:+.2f} {o[1]:+.2f} mm (peg tool y/z, {OFFSET_MODEL.name}): on-axis tests use it")
        if teach is not None:
            self.em = EpisodeMachine()
            inner = teach.st.on_tick
            teach.st.on_tick = lambda: (inner(), self.background())

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
            if c["episode_id"]:
                self.episode_id = c["episode_id"] - 1             # the next episode gets this id
        elif cmd == wire.PAUSE:
            self.paused = True
        elif cmd == wire.RESUME:
            self.paused = False
        elif cmd == wire.ABORT:
            self.abort = True

    def publish(self):
        f = obs_frame(self.io)
        if f is None:
            return False
        self.pub.send(wire.encode_frame(f, self.episode_id, self.step, self.state, self.reason,
                                        self.reward, self.action))
        self.counts["frames"] += 1
        self.last_pub = time.monotonic()
        return True

    def background(self):
        """Every 100 Hz control tick: keep RESET / IDLE / FAULT frames flowing at 15 Hz while
        the robot runs a blocking motion (the reset), keep reading commands, and stop a
        retract that pushes the hole."""
        if self.watch is not None and self.sideways(self.watch) > HOLE_SHIFT_MAX:
            self.watch = None
            raise HoleMoved(f"hole pushed {HOLE_SHIFT_MAX * 1000:.0f}+ mm sideways")
        if self.t is not None and self.state in (wire.FS_POLICY, wire.FS_INTERVENTION, wire.FS_RESET):
            self.guard()                                      # only while something is being commanded
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

    def command_site(self, T_site):
        """Send a commanded IK EE pose, clamped to the box around the hole (base-fixed goal)."""
        arm = self.t.st.arms["right"]
        T_peg = T_site @ np.linalg.inv(arm.map) @ pht.PEG_TOOL
        T_l = self.io.ee("left")
        H = T_l @ pht.HOLE_TOOL
        R_ref = (H @ phs.T_from([0, 0, 0], Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix()))[:3, :3]
        box = SafetyBox(H, [pht.TOP - self.t.a.push - 0.003, -BOX_ACROSS, -BOX_ACROSS],
                        [pht.TOP + BOX_UP, BOX_ACROSS, BOX_ACROSS], [BOX_TILT] * 3, R_ref)
        T_peg, _ = box.clamp(T_peg)
        p = np.linalg.inv(H) @ np.r_[T_peg[:3, 3], 1.0]                   # x = height on the axis
        if np.hypot(p[1] - self.axis_c[0], p[2] - self.axis_c[1]) > ON_AXIS and p[0] < pht.TOP + FLOOR:
            # beside the (calibrated) hole: stay above the rim
            p[0] = pht.TOP + FLOOR
            T_peg = T_peg.copy()
            T_peg[:3, 3] = (H @ p)[:3]
        arm.anchor, arm.goal = None, T_peg @ np.linalg.inv(pht.PEG_TOOL)
        return arm.goal @ arm.map

    # --- episodes -------------------------------------------------------------------------
    def reset(self):
        """RESET frames while the teach tool resets; -> True when the next episode starts."""
        self.state, self.reason, self.reward, self.action = wire.FS_RESET, wire.TR_NONE, 0.0, np.zeros(6)
        try:
            self.retract()
            self.j7_ref, self.j7_since = float(self.io.effort[6]), None        # peg clear: unloaded reference
            ok = self.t.hw_reset()
        except (phr.LagTrip, HoleMoved, GuardTrip) as e:
            print(f"reset stopped: {e}")
            ok = False
        if not ok:
            self.watch, self.state, self.running = None, wire.FS_FAULT, False
            print("reset failed: FAULT, holding (check the robot, then EnvCmd RESET)")
            return False
        self.t.take_right()
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
        self.publish()                                        # step 0: the observation after the reset
        print(f"episode {self.episode_id}: start, peg depth {ins['depth'] * 1000:+.1f} mm")
        return True

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
        T_up = T_peg.copy()
        T_up[:3, 3] += rise * H[:3, 0]
        self.watch = self.hole_now()
        self.retract_floor = abs(float(self.io.effort[6]) - self.j7_ref)
        try:
            self.t.move_right(T_up @ np.linalg.inv(pht.PEG_TOOL), self.t.a.speed, "reset_out")
        finally:
            self.watch, self.retract_floor = None, None

    def episode_tick(self):
        """One 15 Hz frame of an episode: act (policy or machine), let the robot move for the
        rest of the period, measure, tag, publish. -> False once TERMINATED."""
        period, t_end = 1.0 / HZ, time.monotonic() + 1.0 / HZ
        mode = self.em.mode
        if mode == "policy":
            stale = self.delta is None or time.monotonic() - self.delta_t > WATCHDOG_S
            d = np.zeros(6) if stale else clip_delta(self.delta, MAX_TRANS, MAX_ROT)
            self.delta = None
        elif mode == "pull_out":
            d = self.em.pull_out_delta((self.io.ee("left") @ pht.HOLE_TOOL)[:3, 0])
        else:                                                 # trace_back
            target = self.em.restore if self.em.restore is not None else \
                self.t.peg_target(pht.TOP + self.em.cfg.clear, phs.T_from([0.0, *(self.offset_mm / 1000.0)], np.eye(3)))[0] \
                @ self.t.st.arms["right"].map
            d = self.em.toward_delta(self.T_cmd, target)
        self.T_cmd = self.command_site(apply_delta(self.T_cmd, d))
        self.action = d
        tripped = None
        try:
            while time.monotonic() < t_end - phs.CTRL_DT / 2:
                self.t.st.tick(None)
        except (GuardTrip, HoleMoved) as e:                   # gear guard: end the episode at once
            tripped = str(e)
            print(f"  step {self.step + 1}: SAFETY stop: {e}")
        ins, q, amps, lift, s_real, s_cmd = self.measure()
        now = time.monotonic() - self.t0
        blocked = self.block.update(now, s_real, s_cmd) is not None
        safety = tripped is not None or self.edge.update(now, s_real, self.io.raw_js) == "edge_hard" or \
            abs(self.io.effort[6] - self.e0[6]) > self.t.a.hard_j7 or self.abort
        arrived = mode == "trace_back" and self.em.restore is not None and \
            all(e < tol for e, tol in zip(pose_error(self.T_cmd, self.em.restore), (0.001, np.radians(1.0))))
        r = self.em.tick(period, ins, q, amps, lift, self.T_cmd, blocked=blocked, safety=safety, arrived=arrived,
                         hole_shift=self.sideways(self.hole_ref), dj7=abs(float(self.io.effort[6]) - self.j7_ref))
        if self.abort and r["frame_state"] == wire.FS_TERMINATED:
            r["reason"], self.abort, self.paused = wire.TR_ABORT, False, True
        self.step = r["step"]
        self.state, self.reason, self.reward = r["frame_state"], r["reason"], r["reward"]
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
                if self.reset():
                    while self.episode_tick():
                        self.poll_commands()
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


def main():
    ap = pht.build_parser(__doc__, default_out=str(HERE / "recordings" / "peg_hole_serl"))
    ap.add_argument("--port-base", type=int, default=7601)
    ap.add_argument("--duration", type=float, default=None, help="s, then quit (tests)")
    a = ap.parse_args()
    pht.apply_setup(a)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    io = pht.TeachIo(a.allow_motion)
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    t, lift_before = None, None
    try:
        t0 = time.monotonic()
        while (obs_frame(io) is None or io.effort is None or io.ee("left") is None or io.ee("right") is None) \
                and time.monotonic() - t0 < 15.0:
            time.sleep(0.1)
        if obs_frame(io) is None:
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
        srv.running = bool(a.autostart and t is not None)
        print(f"serving at {HZ:.0f} Hz on {a.port_base} (+{wire.PORT_OBS} frames, +{wire.PORT_CONTROL} control); "
              + ("waiting for EnvCmd RESET" if t is not None and not srv.running else
                 "episodes start now" if srv.running else "no motion: IDLE frames only"))
        srv.run(a.duration)
        print(f"done: {srv.counts}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
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
