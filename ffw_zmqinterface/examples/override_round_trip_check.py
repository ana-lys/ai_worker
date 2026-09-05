#!/usr/bin/env python3
"""override_round_trip_check.py -- do the 25 floats I send in an OverrideCmd
come back in the Obs joint block?

Controller-side live-link check against the framed ZMQ gateway (gateway_node /
zmq_spacemouse_gateway, already running whenever spacemouse_unified_teleop is
up). SUBs Obs on the STATE port (6001), PUBs OverrideCmd on the CONTROL port
(6002). It takes the 25 joint positions from the first Obs, sends those exact
25 floats back as an OverrideCmd (both are in JOINT_STATE_NAMES order, so index
N == Obs joint N == dataset q-column N), then per received Obs diffs command[i]
against obs.joint_pos[i] and prints the table -- max |diff| and last diff per
joint over the window.

The OverrideCmd rail's robot-side consumer has landed (ffw_ik_solver_teleop
runs goal_source=ee|rail and apply_rail_sync() on /qpos_rail -- commit 17a6a60),
but the gateway does not yet relay OverrideCmd to /qpos_rail, so nothing
applies this rail end-to-end yet. This client therefore generates a sustained
echo (send the observed 25 back at ~25 Hz) and reads the loop from the client
side of the link:

  CAN prove: our 212-byte OverrideCmd frame is accepted by the gateway SUB
  (no decode / length rejection), the 724-byte Obs stream is well-formed and
  decodes, and the 25-for-25 joint ordering is consistent end to end -- with
  the robot holding still, what comes back in Obs matches what we sent.

  CORROBORATES: the gateway sees BOTH halves -- it diffs every received
  OverrideCmd against the joint block of the last Obs it published (its
  override_check loop check, on by default). Its [override-loop] WARNs during
  the run and its per-joint table + PASS/FAIL verdict at shutdown are the
  server-side answer to the same question this script asks from the client.
  Read both together.

  CANNOT yet prove: the robot tracking a commanded pose -- that needs the
  gateway -> /qpos_rail forward hop wired. Until then run hands OFF the
  SpaceMouse so nothing else is commanding the robot, and read the table as a
  transport + ordering check: a nonzero diff means either drift / another
  commander, or a joint whose Obs index is not the joint the command index
  addresses.

Pure ZMQ client -- no rclpy. Run from the ffw_zmqinterface package root:
    python3 examples/override_round_trip_check.py                # local gateway
    python3 examples/override_round_trip_check.py --host 192.168.0.249
    python3 examples/override_round_trip_check.py --margin 0.01 --seconds 3
"""
import argparse
import os
import sys
import threading
import time

import zmq

# Use the installed package when the workspace is sourced, else the source tree.
try:
    import ffw_zmqinterface.protocol as proto
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "ffw_zmqinterface"))
    import protocol as proto

STATE_PORT = 6001     # gateway PUB: Obs
CONTROL_PORT = 6002   # gateway SUB: ControlCmd + OverrideCmd
DEFAULT_HOST = "127.0.0.1"  # gateway co-located with spacemouse_unified_teleop
SEND_HZ = 25.0        # how often we re-assert the OverrideCmd
STARTUP_S = 0.4       # ZMQ PUB/SUB slow-joiner handshake


def _fmt(v):
    return "-" if v is None else f"{v:+.4f}"


class RoundTrip:
    """Shared state between the Obs drain thread and the main command loop."""

    def __init__(self, n):
        self.n = n
        self.lock = threading.Lock()
        self.first_obs = None        # 25 qpos from the first live Obs (base)
        self.command = None          # the 25 floats currently being sent
        self.ticks = 0               # Obs frames compared since command set
        self.max_abs = [0.0] * n     # max |command[i] - obs.joint_pos[i]|
        self.last_diff = [None] * n  # diff on the most recently compared Obs

    def on_obs(self, obs):
        with self.lock:
            if self.first_obs is None:
                self.first_obs = list(obs.joint_pos)
            if self.command is None:
                return
            self.ticks += 1
            for i in range(self.n):
                d = self.command[i] - obs.joint_pos[i]
                self.last_diff[i] = d
                if abs(d) > self.max_abs[i]:
                    self.max_abs[i] = abs(d)


def zmq_obs_loop(sub, rt, stop):
    """Drain the gateway Obs stream, feeding each frame into rt."""
    while not stop.is_set():
        try:
            frame = sub.recv()
        except zmq.ZMQError:
            break
        try:
            obs, _ts = proto.decode_obs(frame)   # 6001 carries Obs only
        except ValueError:
            continue
        rt.on_obs(obs)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default=os.environ.get("GW_HOST", DEFAULT_HOST),
                    help="gateway host (the machine running gateway_node / "
                         "spacemouse_unified_teleop). Default %(default)s.")
    ap.add_argument("--margin", type=float, default=0.02,
                    help="pass margin per joint, rad/m (default %(default)s).")
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="how long to send + compare (default %(default)s).")
    ap.add_argument("--wait", type=float, default=5.0,
                    help="max seconds to wait for the first Obs "
                         "(default %(default)s).")
    args = ap.parse_args()

    n = proto.N_JOINT_STATE
    names = proto.JOINT_STATE_NAMES
    rt = RoundTrip(n)
    state = f"tcp://{args.host}:{STATE_PORT}"
    control = f"tcp://{args.host}:{CONTROL_PORT}"
    print(f"gateway: obs {state} | override {control}  margin={args.margin}",
          flush=True)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect(state)
    pub = ctx.socket(zmq.PUB)
    pub.connect(control)
    time.sleep(STARTUP_S)  # let the SUB/PUB handshake settle

    stop = threading.Event()
    drain = threading.Thread(target=zmq_obs_loop, args=(sub, rt, stop),
                             daemon=True)
    drain.start()

    # --- wait for the live Obs stream ---------------------------------------
    # The gateway only starts it once BOTH /ik_solver/achieved_ee_pose_{r,l}
    # AND the first /joint_states have been seen.
    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline and rt.first_obs is None:
        time.sleep(0.05)
    with rt.lock:
        base = rt.first_obs
    if base is None:
        print(f"FAIL: no Obs on {state} within {args.wait:.0f}s -- is the "
              f"teleop stack / gateway up (both arms achieved + /joint_states)?",
              flush=True)
        sys.exit(1)

    # --- send the observed 25 back out as the OverrideCmd -------------------
    command = list(base)
    with rt.lock:
        rt.command = command
    print(f"sending back the {n} observed joint positions as OverrideCmd "
          f"({names[0]}..{names[-1]}) for {args.seconds:.0f}s @ "
          f"{SEND_HZ:.0f}Hz ...", flush=True)
    print("hands OFF the SpaceMouse for a clean round trip.", flush=True)

    t0 = time.monotonic()
    interval = 1.0 / SEND_HZ
    while time.monotonic() - t0 < args.seconds:
        pub.send(proto.encode_override(proto.OverrideCmd(command),
                                       ts=time.time()))
        time.sleep(interval)
    time.sleep(0.2)  # let the last Obs land

    stop.set()
    with rt.lock:
        ticks = rt.ticks
        max_abs = list(rt.max_abs)
        last = list(rt.last_diff)
    if ticks == 0:
        print(f"FAIL: no Obs received during the {args.seconds:.0f}s window.",
              flush=True)
        sys.exit(2)

    # --- report: what came back vs what we sent -----------------------------
    print(f"\ncompared {ticks} Obs frames; per-joint max |sent - obs| and last "
          f"diff:\n", flush=True)
    print(f"  {'idx':>3}  {'joint':<20} {'max|diff|':>9} {'last diff':>10}  "
          f"{'verdict':<6}", flush=True)
    over = []
    for i in range(n):
        ok = max_abs[i] <= args.margin
        if not ok:
            over.append(i)
        print(f"  {i:>3}  {names[i]:<20} {max_abs[i]:9.4f} "
              f"{_fmt(last[i]):>10}  {'ok' if ok else 'OVER'}", flush=True)

    print("", flush=True)
    if not over:
        print(f"RESULT: PASS -- the {n} floats that came back in Obs match "
              f"the {n} floats we sent within {args.margin} "
              f"(max |diff| = {max(max_abs):.4f}). Frame accepted, order "
              f"consistent, robot holding.", flush=True)
        sys.exit(0)
    print(f"RESULT: FAIL -- {len(over)} joint(s) came back past the margin "
          f"(max |diff| = {max(max_abs):.4f} > {args.margin}). The gateway does not "
          f"relay OverrideCmd to /qpos_rail yet, so nothing applies this "
          f"rail to the robot -- a diff means the robot is drifting / being "
          f"commanded by something else (SpaceMouse / ControlCmd), or a joint "
          f"whose Obs index is not the joint this command index addresses.",
          flush=True)
    sys.exit(3)


if __name__ == "__main__":
    main()
