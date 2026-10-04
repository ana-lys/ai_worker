"""ZMQ server: the same event handling for the simulator and the real robot.

Ports (offsets from the base, see wire.py; +2 Priv and +3 Record stay the
robot gateway's):
  OBS     +0  PUB  gateway Obs frame after every tick (and after a reset)
  CONTROL +1  SUB  ControlCmdDelta, EnvCmd
  STATUS  +4  PUB  EnvStatus after every tick and every state change
  IMAGES  +5  PUB  both wrist images, after every Obs (multipart)

Events (identical on both backends):
  EnvCmd RESET(seed, episode_id)  state RESETTING -> task.reset(seed) -> READY
                                  (FAULT + reset_ok=0 if no valid sample)
  ControlCmdDelta                 READY/RUNNING: queued for the next tick;
                                  state RUNNING. Ignored in other states.
  tick                            RUNNING: task.step(queued delta, or zero
                                  = hold); terminated/truncated -> DONE
  EnvCmd PAUSE / RESUME           PAUSED holds (deltas ignored) / back to RUNNING
  EnvCmd ABORT                    peg straight up the hole axis to clear the
                                  rim, then DONE
  EnvCmd PING                     status only

Only the clock differs:
  LockstepClock   a tick happens when a delta arrives: one delta = one agent
                  period of sim time, Obs straight back (as fast as the
                  controller sends).
  RealtimeClock   a tick every 1/hz s of wall time; the deltas that arrived
                  since the last tick are composed into one; none = hold.
"""
import argparse
import struct
import time

import numpy as np
import zmq

from . import effort, wire
from .geometry import apply_delta, delta_between

class LockstepClock:
    def timeout_ms(self):
        return 100

    def due(self, have_delta):
        return have_delta

    def ticked(self):
        pass


class RealtimeClock:
    def __init__(self, hz):
        self.period = 1.0 / hz
        self.next = time.monotonic() + self.period

    def timeout_ms(self):
        return max(0, int((self.next - time.monotonic()) * 1000))

    def due(self, have_delta):
        return time.monotonic() >= self.next

    def ticked(self):
        self.next += self.period
        if self.next < time.monotonic():          # fell behind: don't burst to catch up
            self.next = time.monotonic() + self.period


def compose(d1, d2):
    """One delta equal to applying d1 then d2."""
    I = np.eye(4)
    return delta_between(I, apply_delta(apply_delta(I, d1), d2))


class PegHoleServer:
    def __init__(self, task, backend_id, clock, port_base=6001, host="*"):
        self.task, self.backend_id, self.clock = task, backend_id, clock
        self.ctx = zmq.Context.instance()
        self.pub_obs = self.ctx.socket(zmq.PUB)
        self.pub_obs.bind(f"tcp://{host}:{port_base + wire.PORT_OBS}")
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.sub.bind(f"tcp://{host}:{port_base + wire.PORT_CONTROL}")
        self.pub_status = self.ctx.socket(zmq.PUB)
        self.pub_status.bind(f"tcp://{host}:{port_base + wire.PORT_STATUS}")
        self.pub_images = self.ctx.socket(zmq.PUB)
        self.pub_images.setsockopt(zmq.SNDHWM, 4)       # never queue stale images
        self.pub_images.bind(f"tcp://{host}:{port_base + wire.PORT_IMAGES}")
        self.state = wire.IDLE
        self.episode_id = 0
        self.reset_ok = 0
        self.pending = None                       # (right6, left6) composed since the last tick
        self.deltas_received = 0
        self.dropped = 0
        self.last = {"reward": 0.0, "success": 0, "terminated": 0, "truncated": 0, "step": 0}

    # --- events --------------------------------------------------------------------
    def on_delta(self, right7, left7):
        if self.state not in (wire.READY, wire.RUNNING):
            return
        self.deltas_received += 1
        r, l = np.array(right7[:6]), np.array(left7[:6])
        if self.pending is None:
            self.pending = (r, l)
        else:
            self.pending = (compose(self.pending[0], r), compose(self.pending[1], l))
        self.state = wire.RUNNING

    def on_env_cmd(self, c):
        cmd = c["cmd"]
        if cmd == wire.RESET:
            self.state = wire.RESETTING
            self.publish_status()
            self.pending = None
            self.episode_id = c["episode_id"] or self.episode_id + 1
            self.last = {"reward": 0.0, "success": 0, "terminated": 0, "truncated": 0, "step": 0}
            try:
                self.task.reset(seed=c["seed"])
                self.reset_ok, self.state = 1, wire.READY
            except RuntimeError as e:
                print(f"reset failed: {e}")
                self.reset_ok, self.state = 0, wire.FAULT
            self.deltas_received = 0
            self.publish_obs()
        elif cmd == wire.PAUSE and self.state in (wire.READY, wire.RUNNING):
            self.state, self.pending = wire.PAUSED, None
        elif cmd == wire.RESUME and self.state == wire.PAUSED:
            self.state = wire.RUNNING
        elif cmd == wire.ABORT and self.state in (wire.READY, wire.RUNNING, wire.PAUSED):
            self.abort()
            self.state = wire.DONE
        self.publish_status()

    def tick(self):
        right, left = self.pending if self.pending is not None else (np.zeros(6), np.zeros(6))
        self.pending = None
        obs, reward, term, trunc, info = self.task.step(right, left)
        self.last = {"reward": reward, "success": int(info["success"]), "terminated": int(term),
                     "truncated": int(trunc), "step": info["steps"]}
        if term or trunc:
            self.state = wire.DONE
        self.publish_obs(obs)
        self.publish_status(obs)

    def abort(self, clear=0.01, speed=0.005):
        """Peg straight up the hole axis until its tip is `clear` above the rim,
        one tick per step (same on both backends; only the pacing differs)."""
        t = self.task
        for _ in range(400):
            ins = t.insertion_state()
            if ins["depth"] <= -clear:
                break
            up = t.b.tool_pose("left")[:3, 0] * speed       # hole axis (tool x) in base_link
            t.step(np.concatenate([up, np.zeros(3)]), None)
            if isinstance(self.clock, RealtimeClock):
                time.sleep(self.clock.period)

    # --- publishing ----------------------------------------------------------------
    def publish_obs(self, obs=None):
        names, q, dq, tau = self.task.b.all_joints()
        idx = {n: i for i, n in enumerate(names)}
        o = wire.gw.Obs()
        grip_q = {"gripper_r_joint1": self.task.b.gripper["right"], "gripper_l_joint1": self.task.b.gripper["left"]}
        for k, n in enumerate(wire.gw.JOINT_STATE_NAMES):
            if n in grip_q:
                o.joint_pos[k] = float(grip_q[n])
            elif n in idx:
                i = idx[n]
                o.joint_pos[k], o.joint_vel[k] = float(q[i]), float(dq[i])
                aj = effort.arm_joint_index(n)
                if aj is not None:          # amps, the robot's unit: load-cell driving K (effort.py)
                    o.joint_effort[k] = float(effort.amps_from_torque(aj[0], tau[i], aj[1]))
        ee = []
        for side in ("right", "left"):                  # Obs ee0 = right, ee1 = left
            T = self.task.b.site_pose(side)
            w, x, y, z = _mat_to_quat_wxyz(T[:3, :3])
            ee.append((*T[:3, 3], *wire.gw.quat_to_rpy(w, x, y, z)))
        o.ee = (tuple(float(v) for v in ee[0]), tuple(float(v) for v in ee[1]))
        o.gripper = tuple(float(v) for v in self.task.b.gripper_normalized())
        self.pub_obs.send(wire.gw.encode_obs(o))
        imgs = self.task.b.render()
        if imgs:
            self.pub_images.send_multipart(wire.encode_images(self.episode_id, self.last["step"],
                                                              imgs["right"], imgs["left"]))

    def publish_status(self, obs=None):
        ins = self.task.insertion_state() if self.state != wire.IDLE else {"depth": 0.0, "lateral": 0.0}
        s = {"state": self.state, "backend": self.backend_id, "reset_ok": self.reset_ok,
             "episode_id": self.episode_id, "deltas_received": self.deltas_received,
             "depth": ins["depth"], "lateral": ins["lateral"],
             "contact_force": self.task.reward.push_back, **self.last}
        self.pub_status.send(wire.encode_status(s))

    # --- loop ------------------------------------------------------------------------
    def run(self):
        poller = zmq.Poller()
        poller.register(self.sub, zmq.POLLIN)
        print(f"peg-hole server: backend {'SIM' if self.backend_id == wire.BACKEND_SIM else 'REAL'}, "
              f"{type(self.clock).__name__}")
        while True:
            have_delta = False
            if poller.poll(self.clock.timeout_ms()):
                while True:
                    try:
                        data = self.sub.recv(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    try:
                        t = wire.msg_type(data)
                        if t == wire.MSG_CONTROL_DELTA:
                            r, l, _ = wire.decode_delta(data)
                            if not all(np.isfinite(r[:6])) or not all(np.isfinite(l[:6])):
                                raise ValueError("non-finite delta")
                            self.on_delta(r, l)
                            have_delta = self.state == wire.RUNNING
                            if isinstance(self.clock, LockstepClock) and have_delta:
                                break                     # lockstep: one delta = one tick
                        elif t == wire.MSG_ENV_CMD:
                            c, _ = wire.decode_env_cmd(data)
                            self.on_env_cmd(c)
                        else:
                            raise ValueError(f"unknown message type {t}")
                    except (ValueError, struct.error) as e:
                        self.dropped += 1
                        print(f"dropped a message ({e}); {self.dropped} so far")
            if self.clock.due(have_delta):
                if self.state == wire.RUNNING:
                    self.tick()
                self.clock.ticked()


def _mat_to_quat_wxyz(R):
    from scipy.spatial.transform import Rotation as Rot
    x, y, z, w = Rot.from_matrix(R).as_quat()
    return w, x, y, z


def main():
    from .peg_hole import PegHoleConfig, PegHoleTask
    from .sim_backend import DEFAULT_LIFT, MujocoBackend
    ap = argparse.ArgumentParser(description="Peg-hole ZMQ server (simulation backend).")
    ap.add_argument("--port-base", type=int, default=6001)
    ap.add_argument("--realtime", action="store_true", help="15 Hz wall-clock ticks instead of lockstep")
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--timestep", type=float, default=0.004, help="physics timestep [s] (implicitfast)")
    ap.add_argument("--no-cameras", action="store_true", help="don't render or publish wrist images")
    ap.add_argument("--lift", type=float, default=DEFAULT_LIFT, help="lift_joint [m], held fixed (0 = top)")
    ap.add_argument("--safety-frame", choices=["hole", "hole_live", "base", "pose", "off"], default="hole",
                    help="frame of both arms' safe range (PegHoleConfig.safety_*); off = no limit")
    ap.add_argument("--safety-point", choices=["tool", "ee"], default="tool")
    a = ap.parse_args()
    backend = MujocoBackend(hz=a.hz, view=a.view, timestep=a.timestep, cameras=not a.no_cameras, lift=a.lift)
    task = PegHoleTask(backend, PegHoleConfig(hz=a.hz, safety=a.safety_frame != "off",
                                              safety_frame="hole" if a.safety_frame == "off" else a.safety_frame,
                                              safety_point=a.safety_point))
    clock = RealtimeClock(a.hz) if a.realtime else LockstepClock()
    PegHoleServer(task, wire.BACKEND_SIM, clock, port_base=a.port_base).run()


if __name__ == "__main__":
    main()
