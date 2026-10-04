#!/usr/bin/env python3
"""Hold at a random pose until told, then run the scripted expert once (diagnosis tool).

Reset to a new random pair (the server's reset: pull out, hole move, seat, peg start), then HOLD
(arms streamed, gear guard on) until a trigger file appears in --trigger-dir:
  expert   run the expert insertion from here (one episode, saved with images)
  reset    another random reset, then hold again
  quit     pull out and stop
After the expert the peg is pulled out and it holds again.

  .venv/bin/python peg_hole_expert_poke.py --allow-motion --trigger-dir /tmp/poke
  touch /tmp/poke/expert
"""
import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import rclpy.signals
from rclpy.executors import SingleThreadedExecutor

import peg_hole_random as phr
import peg_hole_serl_server as srvmod
import peg_hole_teach as pht
from peg_hole_demo_recorder import DemoServer
from peg_hole_serl_server import EXPERT_CFG, Expert
from ffw_peg_hole_env import images, wire

HERE = Path(__file__).resolve().parent


def take(trig):
    for name in ("expert", "reset", "quit"):
        p = Path(trig) / name
        if p.exists():
            p.unlink()
            return name
    return None


def hold(srv, trig):
    print(f"HOLDING -- touch {trig}/expert | reset | quit", flush=True)
    n = 0
    while True:
        srv.t.st.tick(None)
        n += 1
        if n % 10 == 0 and srv.state == wire.FS_POLICY:
            srv.measure()                                 # peg clear: re-zero the gear guard / hole watch (a
        k = take(trig)                                    # static wrist current drifts ~+450 mA a minute)
        if k:
            return k


def run_expert(srv, out):
    """One expert episode from the current pose (the episode reset() started)."""
    srv.buf = []
    ex = Expert(srv, EXPERT_CFG, srv.offset_mm)
    while True:
        srv.delta, srv.delta_t = ex.delta(), time.monotonic()
        if not srv.episode_tick():
            break
    info = dict(srv.info)
    res = {"reason": wire.TR_NAMES[srv.reason], "steps": srv.step, "push_back_peak": float(info.get("push_back_peak", 0)),
           "return": float(info.get("return", 0)), "offset_mm": srv.offset_mm.tolist(),
           "time": datetime.now().isoformat(timespec="seconds")}
    path = out / f"expert_{datetime.now().strftime('%H%M%S')}_{res['reason']}.npz"
    b = srv.buf
    np.savez_compressed(path, frames=np.array([x[0] for x in b], np.uint8).reshape(-1, wire.FRAME_BYTES),
                        img_right=np.array([x[1] for x in b], np.uint8), img_left=np.array([x[2] for x in b], np.uint8),
                        img_ok=np.array([x[3] for x in b], bool), meta=np.array(json.dumps(res)))
    srv.buf = None
    print(f"EXPERT: {res['reason']} at step {res['steps']}, peak {res['push_back_peak']:.1f} N -> {path}", flush=True)
    srv.after_episode()


def main():
    ap = pht.build_parser(__doc__, default_out=str(HERE / "recordings" / "peg_hole_poke" / "teach"))
    srvmod.add_seat_args(ap)
    ap.add_argument("--trigger-dir", default="/tmp/peg_hole_poke")
    ap.add_argument("--out-dir", default=str(HERE / "recordings" / "peg_hole_poke"))
    ap.add_argument("--port-base", type=int, default=7621)
    a = ap.parse_args()
    if not a.allow_motion:
        ap.error("pass --allow-motion")
    pht.apply_setup(a)
    os.makedirs(a.trigger_dir, exist_ok=True)
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    io = pht.TeachIo(True)
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    threading.Thread(target=ex.spin, daemon=True).start()
    t, cams = None, None
    try:
        t0 = time.monotonic()
        while (srvmod.gateway_obs(io) is None or io.effort is None or io.ee("left") is None
               or io.ee("right") is None) and time.monotonic() - t0 < 15.0:
            time.sleep(0.1)
        cams = images.WristCameras()
        t = pht.Teach(io, a)
        t.set_tool_model(a.tool_model)
        t.st.arms["left"] = phr.Arm("left", io.ee("left"), io.latest_site("left"))
        io.set_hold_side("left", True)
        t.take_right()
        srv = DemoServer(io, a.port_base, t)
        srv.cams, srv.seat, srv.seat_path = cams, srvmod.seat_from_args(a), srvmod.path_from_args(a)
        srv.reset_mode = wire.RESET_RANDOM
        cmd = "reset"
        while cmd != "quit":
            if cmd == "reset":
                if not srv.reset():
                    print("RESET FAILED (FAULT) -- check the robot", flush=True)
                else:
                    ins = srv.measure()[0]
                    print(f"AT RANDOM POSE: episode {srv.episode_id}, peg {-ins['depth'] * 1000:.1f} mm above the rim, "
                          f"{ins['lateral'] * 1000:.1f} mm off the axis, tilt {ins['tilt_deg']:.1f} deg", flush=True)
            elif cmd == "expert":
                if srv.state == wire.FS_POLICY:
                    run_expert(srv, out)
                else:
                    print("no episode to run the expert in -- reset first", flush=True)
            cmd = hold(srv, a.trigger_dir)
        srv.after_episode()
    except (KeyboardInterrupt, pht.Abort) as e:
        print(f"stopped ({type(e).__name__})")
    finally:
        if cams is not None:
            cams.stop()
        if t is not None:
            t.summary.close()
        io.set_hold(False)
        print("both arms released to the SpaceMouse", flush=True)
        time.sleep(0.2)
        ex.shutdown()
        io.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
