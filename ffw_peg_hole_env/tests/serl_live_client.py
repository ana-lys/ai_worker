#!/usr/bin/env python3
"""Live test client for peg_hole_serl_server.py (real robot): EnvCmd RESET, then answer
every POLICY frame with a fixed delta, log every frame's tag, summarise per episode.

  --policy hold   zero deltas: episodes end by TIMEOUT (checks RESET -> POLICY -> TERMINATED)
  --policy down   straight down base_link -z at --speed m/tick: the peg lands wherever it
                  starts, so it exercises the machine intervention and the terminations

  ../ffw_collision_checker/scripts/.venv/bin/python tests/serl_live_client.py --policy down --episodes 2
"""
import argparse
import collections
import sys
import time
from pathlib import Path

import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import wire  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port-base", type=int, default=7601)
    ap.add_argument("--policy", choices=["hold", "down"], default="hold")
    ap.add_argument("--speed", type=float, default=0.0015, help="m per tick for --policy down")
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--timeout", type=float, default=300.0, help="s for the whole test")
    a = ap.parse_args()
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect(f"tcp://{a.host}:{a.port_base + wire.PORT_OBS}")
    pub = ctx.socket(zmq.PUB)
    pub.connect(f"tcp://{a.host}:{a.port_base + wire.PORT_CONTROL}")
    time.sleep(0.5)                                           # PUB/SUB join
    pub.send(wire.encode_env_cmd(wire.RESET, seed=1, episode_id=1))
    delta = np.zeros(6) if a.policy == "hold" else np.array([0.0, 0.0, -a.speed, 0.0, 0.0, 0.0])
    t0, done, last = time.time(), [], None
    runs = collections.Counter()
    ep = {"states": collections.Counter(), "reward": 0.0}
    while time.time() - t0 < a.timeout and len(done) < a.episodes:
        if not sub.poll(500):
            continue
        obs, tag, _ = wire.decode_frame(sub.recv())
        st = tag["frame_state"]
        key = (tag["episode_id"], st)
        if key != last:                                       # log state changes, not every frame
            print(f"[{time.time() - t0:6.1f} s] episode {tag['episode_id']} step {tag['step']:3d}: {wire.FS_NAMES[st]}"
                  + (f" {wire.TR_NAMES[tag['reason']]}" if st == wire.FS_TERMINATED else "")
                  + (f"  action {np.round(np.array(tag['action'][:3]) * 1000, 2)} mm" if st == wire.FS_INTERVENTION else ""))
            last = key
        if st in (wire.FS_POLICY, wire.FS_INTERVENTION, wire.FS_TERMINATED):
            ep["states"][wire.FS_NAMES[st]] += 1
            ep["reward"] += tag["reward"]
        runs[wire.FS_NAMES[st]] += 1
        if st == wire.FS_POLICY:
            pub.send(wire.encode_delta((*delta, 0.0), (0.0,) * 7))
        if st == wire.FS_TERMINATED:
            done.append((tag["episode_id"], wire.TR_NAMES[tag["reason"]], dict(ep["states"]), ep["reward"]))
            ep = {"states": collections.Counter(), "reward": 0.0}
        if st == wire.FS_FAULT:
            print("server FAULT")
            break
    pub.send(wire.encode_env_cmd(wire.PAUSE))
    print("\nframes by state:", dict(runs))
    for e in done:
        print(f"episode {e[0]}: {e[1]}, frames {e[2]}, return {e[3]:+.3f}")


if __name__ == "__main__":
    main()
