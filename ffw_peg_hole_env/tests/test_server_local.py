#!/usr/bin/env python3
"""Local end-to-end test of the real-robot SERL server -- no robot, no ROS nodes.

Runs the REAL peg_hole_serl_server.SerlServer (episode logic, resets, wire, run loop) against a
fake robot: the right arm reaches every goal instantly, the hole sits still with its insertion
axis = base +z, and the push-back force is scripted (7.5 N whenever the tip is below the rim
more than 1.5 mm off the axis, i.e. binding). A real ZMQ client talks to it on localhost, the
way a HIL-SERL client would, and checks the episode protocol:

  1. RESET AUTO -> new pair; straight down off the axis -> TERMINATED BIND, negative reward
  2. the server pulls the peg out by itself and waits (IDLE)
  3. RESET AUTO -> a retry (reset_kind 1, parent = 1) starting at the last good state
     (tip >= 3 mm above the rim, where episode 1 left it); align + insert -> SUCCESS
  4. RESET AUTO after a success -> new pair
  5. fail it 4 times -> 3 retries, then a new pair (the retry cap)
  6. RESET RANDOM after a failure -> new pair
  7. --failure-mode intervene: the same binding -> the policy's frame gets the fail penalty, the
     machine pulls out and the expert (the demo recorder's) inserts: INTERVENTION frames, SUCCESS
  every frame: type 14, no NaN; step 0 has action 0.

  source /opt/ros/jazzy/setup.bash   # the server imports rclpy / the gateway module
  ../ffw_collision_checker/scripts/.venv/bin/python tests/test_server_local.py
"""
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import zmq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "ffw_collision_checker" / "scripts"))
import peg_hole_random as phr  # noqa: E402
import peg_hole_serl_server as S  # noqa: E402
import peg_hole_stroke as phs  # noqa: E402
import peg_hole_teach as pht  # noqa: E402
from ffw_peg_hole_env import wire  # noqa: E402

PORT = 7690
failures = []


def check(name, cond):
    print(f"{'ok  ' if cond else 'FAIL'} {name}")
    if not cond:
        failures.append(name)


# --- fake robot ----------------------------------------------------------------------------------
R_H = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])    # hole tool frame: x (axis) = base +z
H = phs.T_from([0.5, 0.1, 0.85], R_H)
NAMES = list(wire.JOINT_STATE_NAMES)


class FakeIo:
    def __init__(self):
        self.left = H @ np.linalg.inv(pht.HOLE_TOOL)
        self.right = np.eye(4)
        self.effort = np.full(7, 100.0)
        self.raw_js = (None, NAMES, [0.0] * 20 + [-0.3] + [0.0] * 4, [0.0] * 25, [10.0] * 25)
        self.tf_buffer = types.SimpleNamespace(lookup_transform=self._no_tf)

    @staticmethod
    def _no_tf(*_):
        raise LookupError("no TF in the fake")

    def ee(self, arm):
        return (self.left if arm == "left" else self.right).copy()

    def latest_site(self, arm):
        return self.ee(arm)                       # IK site = TF EE: map = identity


class FakeStreamer:
    def __init__(self, io):
        self.io, self.arms, self.on_tick = io, {}, lambda: None

    def _sync(self):
        arm = self.arms.get("right")
        if arm is not None:
            T = arm.goal if arm.anchor is None else self.io.left @ np.linalg.inv(arm.anchor) @ arm.goal
            self.io.right, arm.last_goal_eff = T.copy(), T.copy()

    def tick(self, _lag):
        self._sync()
        self.on_tick()
        time.sleep(0.002)

    def move(self, name, T, speed, lag, anchor=None):
        self.arms[name].goal, self.arms[name].anchor = T.copy(), None
        self._sync()

    def settle(self, *_):
        self._sync()


class FakeTeach:
    """The parts of peg_hole_teach.Teach the server uses."""

    def __init__(self, io):
        self.io, self.st = io, FakeStreamer(io)
        self.a = types.SimpleNamespace(speed=0.01, move_speed=0.05, clear=0.02, push=0.035, edge_force=4.0,
                                       hard_edge=25.0, hard_j7=450.0, reach_lag=0.015, settle_timeout=5.0,
                                       start_xy=None)
        self.rng = np.random.default_rng(0)
        self.start_lateral = 0.008               # every new pair: the peg 8 mm off the axis, 30 mm up
        self.new_pairs = 0

    peg_target = pht.Teach.peg_target

    def phase(self, _):
        pass

    def held(self, _):
        self.st.tick(None)

    def take_right(self):
        if "right" not in self.st.arms:
            self.st.arms["right"] = phr.Arm("right", self.io.ee("right"), self.io.latest_site("right"))

    def move_right(self, T, speed, phase):
        self.take_right()
        self.st.move("right", T, speed, None)

    def hw_reset(self, pair=None):
        self.new_pairs += 1
        T, _ = self.peg_target(pht.TOP + 0.03, phs.T_from([0.0, self.start_lateral, 0.0], np.eye(3)))
        self.move_right(T, self.a.move_speed, "reset_peg")
        return True

    def go_start(self, tries=10):
        return True


def scripted_force(srv):
    """Binding: 7.5 N with the tip below the rim > 1.5 mm off the axis (the BIND rule is 7 N x 2)."""
    rw = srv.em.reward

    def f(ins, q, amps, lift):
        rw.ref, rw.f_axial = 0.0, 0.0
        rw.push_back = 7.5 if ins["depth"] > 0.0005 and ins["lateral"] > 0.0015 else 0.0
        return rw.push_back
    return f


# --- client --------------------------------------------------------------------------------------
class Client:
    def __init__(self):
        ctx = zmq.Context.instance()
        self.sub = ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.sub.connect(f"tcp://127.0.0.1:{PORT + wire.PORT_OBS}")
        self.pub = ctx.socket(zmq.PUB)
        self.pub.connect(f"tcp://127.0.0.1:{PORT + wire.PORT_CONTROL}")
        self.bad_frames, self.frames = 0, 0
        self.pairs, self.paired = 0, 0                    # lockstep: frame k+1's tag.action == reply to frame k
        time.sleep(0.5)

    def recv(self, timeout=10.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if self.sub.poll(100):
                parts = self.sub.recv_multipart()
                obs, priv, tag, ts, _ = wire.decode_frame_parts(parts)
                self.frames += 1
                if wire.msg_type(parts[0]) != wire.MSG_FRAME or not (np.isfinite(obs["vector"]).all()
                                                                     and np.isfinite(priv["vector"]).all()):
                    self.bad_frames += 1
                return obs, priv, tag
        raise TimeoutError("no frame")

    def reset(self, mode):
        """env.reset(): RESET(mode), then the next episode's step 0 -> (priv, tag) of step 0."""
        self.pub.send(wire.encode_env_cmd(wire.RESET, params=(float(mode),) + (0.0,) * 7))
        while True:
            obs, priv, tag = self.recv()
            if tag["frame_state"] == wire.FS_POLICY and tag["step"] == 0:
                return priv, tag

    def episode(self, policy):
        """Run one episode -> (TERMINATED tag, its priv, step-0 priv, the IDLE priv after it)."""
        sent, self.trace = None, []
        while True:
            obs, priv, tag = self.recv()
            self.trace.append((tag["frame_state"], tag["reward"], np.array(tag["action"]), priv["mode"]))
            if tag["frame_state"] == wire.FS_INTERVENTION:
                sent = None                                   # the machine drives: nothing of ours to pair
            if sent is not None and tag["frame_state"] in (wire.FS_POLICY, wire.FS_TERMINATED):
                self.pairs += 1
                self.paired += bool(np.allclose(tag["action"], np.clip(sent, -S.MAX_TRANS, S.MAX_TRANS)[:6], atol=1e-9))
            if tag["frame_state"] == wire.FS_TERMINATED:
                end_tag, end_priv = tag, priv
                break
            if tag["frame_state"] == wire.FS_POLICY:
                sent = np.array(policy(priv))
                self.pub.send(wire.encode_delta((*sent, 0.0), (0.0,) * 7))
        while True:                                                       # the server pulls out, then IDLE
            obs, priv, tag = self.recv()
            if tag["frame_state"] == wire.FS_IDLE:
                return end_tag, end_priv, priv


def down(_priv):
    return (0.0, 0.0, -0.002, 0.0, 0.0, 0.0)                              # straight down base -z (into the hole)


def insert(priv):
    """Align on the axis (hole y = base y, hole z = base -x), then down it."""
    ey, ez = priv["lateral_yz"]
    dz = -0.002 if np.hypot(ey, ez) < 0.0005 else 0.0
    return (float(np.clip(ez, -0.004, 0.004)), float(np.clip(-ey, -0.004, 0.004)), dz, 0.0, 0.0, 0.0)


def main():
    io = FakeIo()
    t = FakeTeach(io)
    t.take_right()
    pht.EdgeForce = lambda *a, **k: types.SimpleNamespace(update=lambda *a: None)   # no current model
    srv = S.SerlServer(io, PORT, t, host="127.0.0.1")
    srv.set_offset([0.0, 0.0])                                          # the fake hole axis is exact
    srv.em.reward.push_back_force = scripted_force(srv)
    threading.Thread(target=srv.run, kwargs={"duration": 600.0}, daemon=True).start()
    c = Client()

    # 1 -- a new pair, failed by binding
    p0, t0 = c.reset(wire.RESET_AUTO)
    check("1 RESET AUTO (nothing before): new pair, step 0 action 0",
          p0["reset_kind"] == 0 and np.allclose(t0["action"], 0) and t.new_pairs == 1)
    end, endp, idle = c.episode(down)
    check(f"1 straight down 8 mm off the axis: TERMINATED BIND, negative reward ({end['reward']:+.3f})",
          end["reason"] == wire.TR_BIND and end["reward"] < -0.4 and endp["r_fail"] < -0.4)
    check(f"2 the server pulled the peg out by itself and waits: IDLE, tip {-idle['depth'] * 1000:.1f} mm above the rim",
          idle["depth"] < -0.015)
    ep1 = end["episode_id"]

    # 3 -- the retry from the last good state, succeeds
    p1, t1 = c.reset(wire.RESET_AUTO)
    check(f"3 RESET AUTO after BIND: retry (kind {p1['reset_kind']:.0f}, count {p1['retry_count']:.0f}, "
          f"parent {p1['parent_episode']:.0f})",
          p1["reset_kind"] == 1 and p1["retry_count"] == 1 and p1["parent_episode"] == ep1 and t.new_pairs == 1)
    check(f"3 it starts at the last good state: tip {-p1['depth'] * 1000:.1f} mm above the rim (>= 3), "
          f"{p1['lateral'] * 1000:.1f} mm off the axis (episode 1 was 8)",
          -0.0045 <= p1["depth"] <= -0.003 and abs(p1["lateral"] - 0.008) < 0.0005)
    end, endp, _ = c.episode(insert)
    check(f"3 the retry, aligned and inserted: SUCCESS, positive reward ({end['reward']:+.3f})",
          end["reason"] == wire.TR_SUCCESS and end["reward"] > 0.9)

    # 4 -- after a success: a new pair
    p2, _ = c.reset(wire.RESET_AUTO)
    check("4 RESET AUTO after SUCCESS: new pair", p2["reset_kind"] == 0 and t.new_pairs == 2)

    # 5 -- the retry cap: fail, 3 retries, then a new pair
    kinds = []
    end, _, _ = c.episode(down)
    for _ in range(3):
        p, _ = c.reset(wire.RESET_AUTO)
        kinds.append((p["reset_kind"], p["retry_count"]))
        end, _, _ = c.episode(down)
    p, _ = c.reset(wire.RESET_AUTO)
    check(f"5 three failing retries {[(int(a), int(b)) for a, b in kinds]}, then a new pair",
          kinds == [(1, 1), (1, 2), (1, 3)] and p["reset_kind"] == 0 and t.new_pairs == 3)

    # 6 -- RESET RANDOM after a failure
    end, _, _ = c.episode(down)
    p, _ = c.reset(wire.RESET_RANDOM)
    check("6 RESET RANDOM after a failure: new pair", end["reason"] == wire.TR_BIND and p["reset_kind"] == 0
          and t.new_pairs == 4)
    # 7 -- a SERL intervention: penalty on the bad state, the machine expert drives to the goal
    srv.em.cfg.failure_mode = "intervene"                  # read at the trigger: applies to the episode 6 started
    end, endp, _ = c.episode(down)
    st = [x[0] for x in c.trace]
    k_int = st.index(wire.FS_INTERVENTION) if wire.FS_INTERVENTION in st else None
    check("7 intervene: binding -> INTERVENTION frames (machine), then TERMINATED SUCCESS in the same episode",
          k_int is not None and end["reason"] == wire.TR_SUCCESS and st[-1] == wire.FS_TERMINATED
          and all(x == wire.FS_INTERVENTION for x in st[k_int:-1]) and endp["interventions"] == 1)
    if k_int is not None:
        r_trig = c.trace[k_int - 1][1]
        modes = {int(x[3]) for x in c.trace[k_int:-1]}
        n_int = len(st) - 1 - k_int
        check(f"7 the policy's last frame (the bad state) carries the fail penalty ({r_trig:+.3f}); the machine's "
              f"{n_int} frames move it (modes {sorted(modes)} = pull_out, expert)",
              r_trig < -0.4 and modes <= {wire.MODE_CODES["pull_out"], wire.MODE_CODES["expert"]}
              and np.mean([np.abs(x[2]).max() > 0 for x in c.trace[k_int:-1]]) > 0.8)
        check(f"7 the episode ends with the success reward ({end['reward']:+.3f})", end["reward"] > 0.9)
    check(f"every frame type {wire.MSG_FRAME}, finite ({c.frames} frames)", c.bad_frames == 0 and c.frames > 100)
    check(f"lockstep: frame k+1's tag.action is the client's reply to frame k ({c.paired}/{c.pairs})",
          c.pairs > 50 and c.paired == c.pairs)
    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: {failures}'}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
