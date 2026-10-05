#!/usr/bin/env python3
"""The REAL peg-hole SERL server code, on the network, driving a FAKE robot -- for a client's dry run.

Same wire, episode logic, resets / retries, lockstep and multipart images as the robot
(peg_hole_serl_server.SerlServer); the robot is test_server_local's fake: the right arm reaches
every goal instantly, the hole sits still (its axis = base +z), the push-back is scripted (7.5 N
when the tip is below the rim more than 1.5 mm off the axis -> BIND after 2 ticks). Images are a
synthetic 128x128 test pattern (step number, episode, a bar that follows the peg depth) so the
client's [Frame, right, left] decode path runs. Nothing moves.

  source /opt/ros/jazzy/setup.bash
  ../ffw_collision_checker/scripts/.venv/bin/python tests/fake_robot_server.py --port-base 7601
  # client: SUB tcp://<this host>:7601, PUB tcp://<this host>:7602
"""
import argparse
import sys
import time
import types
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_server_local as L  # noqa: E402  (the fake robot)
import peg_hole_serl_server as S  # noqa: E402
import peg_hole_teach as pht  # noqa: E402
from ffw_peg_hole_env import wire  # noqa: E402


class FakeCams:
    """Synthetic wrist images: a test pattern with the episode / step and the peg depth."""

    def __init__(self, srv):
        self.srv = srv

    def grab(self):
        imgs, ts = {}, {}
        H = self.srv.io.ee("left") @ pht.HOLE_TOOL
        depth = pht.TOP - (np.linalg.inv(H) @ self.srv.io.ee("right") @ pht.PEG_TOOL)[0, 3]
        for k, cam in enumerate(("right", "left")):
            im = np.full((wire.IMG, wire.IMG, 3), (40, 40 + 60 * k, 90), np.uint8)
            y = int(np.clip(64 + depth * 1000, 0, 127))
            cv2.rectangle(im, (54, 0), (74, y), (230, 230, 230), -1)
            cv2.putText(im, cam[0].upper(), (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            cv2.putText(im, f"e{self.srv.episode_id} s{self.srv.step}", (4, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (255, 255, 0), 1)
            imgs[cam], ts[cam] = im, time.time()
        return imgs, ts

    def stop(self):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port-base", type=int, default=7601)
    ap.add_argument("--host", default="*", help="bind address (* = all interfaces)")
    ap.add_argument("--failure-mode", choices=("terminate", "intervene"), default="terminate")
    ap.add_argument("--auto-reset", action="store_true")
    ap.add_argument("--no-images", action="store_true")
    a = ap.parse_args()
    io = L.FakeIo()
    t = L.FakeTeach(io)
    t.take_right()
    pht.EdgeForce = lambda *x, **k: types.SimpleNamespace(update=lambda *y: None)
    srv = S.SerlServer(io, a.port_base, t, host=a.host)
    srv.set_offset([0.0, 0.0])
    srv.em.reward.push_back_force = L.scripted_force(srv)
    srv.em.cfg.failure_mode, srv.auto_reset = a.failure_mode, a.auto_reset
    if not a.no_images:
        srv.cams = FakeCams(srv)
    print(f"FAKE ROBOT -- the real server code on {a.host}:{a.port_base} (+0 frames{' + images' if srv.cams else ''}, "
          f"+1 control), failure mode {a.failure_mode}; waiting for EnvCmd RESET", flush=True)
    try:
        srv.run()
    except KeyboardInterrupt:
        print(f"\nstopped: {srv.counts}")


if __name__ == "__main__":
    main()
