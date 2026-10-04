#!/usr/bin/env python3
"""Unit checks for episode.EpisodeMachine on synthetic traces (push-back scripted,
no robot, no MuJoCo inputs needed).

  ../ffw_collision_checker/scripts/.venv/bin/python tests/test_episode.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import wire  # noqa: E402
from ffw_peg_hole_env.episode import EpisodeMachine  # noqa: E402
from ffw_peg_hole_env.geometry import T_from  # noqa: E402

failures = []
DT = 1 / 15


def check(name, cond):
    print(f"{'ok  ' if cond else 'FAIL'} {name}")
    if not cond:
        failures.append(name)


def machine():
    m = EpisodeMachine()
    m.force = [0.0]                                   # scripted push-back for the next tick
    m.reward.push_back_force = lambda ins, q, amps, lift: m.force[0]
    return m


def ins(depth_mm, lateral_mm=0.0):
    return {"depth": depth_mm / 1000, "lateral": lateral_mm / 1000, "across": (lateral_mm / 1000, 0.0),
            "tilt_deg": 0.0}


def pose(depth_mm):
    return T_from([0.5, 0.0, 0.9 - depth_mm / 1000], np.eye(3))


def run(m, trace, **kw):
    """trace: [(depth_mm, force_N, lateral_mm)] -> list of tick results."""
    out = []
    for d, f, lat in trace:
        m.force[0] = f
        out.append(m.tick(DT, ins(d, lat), None, None, None, pose(d), **kw))
        if out[-1]["mode"] == "reset":
            break
    return out


def test_clean_success():
    m = machine()
    m.start(1, ins(-20), None, None, None, pose(-20))
    r = run(m, [(d, 2.0, 0.0) for d in np.arange(-19, 40, 2.0)])
    states = [x["frame_state"] for x in r]
    check("clean push: POLICY frames then one TERMINATED SUCCESS",
          states[:-1] == [wire.FS_POLICY] * (len(r) - 1) and r[-1]["frame_state"] == wire.FS_TERMINATED
          and r[-1]["reason"] == wire.TR_SUCCESS and r[-1]["mode"] == "reset")
    check("clean push: success at 34 mm, reward 1.4 (2 N peak)", abs(r[-1]["terms"]["success"] - 1.4) < 1e-9)


def test_intervention_cycle():
    m = machine()
    m.start(2, ins(-20), None, None, None, pose(-20))
    trace = [(d, 2.0, 0.0) for d in (-15, -10, -6, -3, 0, 4, 8)] + [(10, 7.0, 0.0)]   # binds at 10 mm
    r = run(m, trace)
    check("binding at 7 N: frame still POLICY (the policy drove it), next mode pull_out",
          r[-1]["frame_state"] == wire.FS_POLICY and r[-1]["mode"] == "pull_out" and r[-1]["interventions"] == 1)
    check("restore pose = the last policy pose near the hole top (tip 3 mm above the rim)",
          np.allclose(m.restore, pose(-3)))
    r = run(m, [(d, 0.0, 0.0) for d in (6, 2, -2, -6)])
    check("pulling out: INTERVENTION frames, trace_back once the tip is 5 mm clear",
          all(x["frame_state"] == wire.FS_INTERVENTION for x in r) and r[-1]["mode"] == "trace_back"
          and [x["mode"] for x in r[:-1]] == ["pull_out"] * 3)
    r1 = m.tick(DT, ins(-4), None, None, None, pose(-4))
    r2 = m.tick(DT, ins(-3), None, None, None, pose(-3), arrived=True)
    check("tracing back: INTERVENTION frames, policy again on arrival",
          r1["frame_state"] == wire.FS_INTERVENTION and r1["mode"] == "trace_back"
          and r2["frame_state"] == wire.FS_INTERVENTION and r2["mode"] == "policy")
    r3 = m.tick(DT, ins(0), None, None, None, pose(0))
    check("after the intervention: POLICY frames, same episode", r3["frame_state"] == wire.FS_POLICY and m.episode_id == 2)


def test_intervention_limit():
    m = machine()
    m.start(3, ins(-20), None, None, None, pose(-20))
    last = None
    for k in range(4):
        r = run(m, [(-3, 0.0, 0.0), (5, 7.0, 0.0)])
        last = r[-1]
        if last["mode"] == "reset":
            break
        run(m, [(-6, 0.0, 0.0)])                                     # pulled out
        m.tick(DT, ins(-3), None, None, None, pose(-3), arrived=True)    # traced back
    check("4th binding with 3 interventions used: TERMINATED INTERVENTIONS, fail priced",
          k == 3 and last["frame_state"] == wire.FS_TERMINATED and last["reason"] == wire.TR_INTERVENTIONS
          and last["terms"]["fail"] < -0.5)


def test_other_terminations():
    m = machine()
    m.start(4, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-20, 0.0, 0.0)] * 400)
    check("no progress: TERMINATED TIMEOUT after 20 s, no fail penalty",
          r[-1]["reason"] == wire.TR_TIMEOUT and r[-1]["step"] == 300 and r[-1]["terms"]["fail"] == 0.0)
    m.start(5, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-5, 0.0, 0.0)], safety=True)
    check("live safety stop: TERMINATED SAFETY", r[-1]["reason"] == wire.TR_SAFETY and r[-1]["frame_state"] == wire.FS_TERMINATED)
    m.start(6, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-3, 0.0, 0.0), (-0.5, 1.0, 4.0)])
    check("4 mm off the axis just above the rim: machine steps in", r[-1]["mode"] == "pull_out")
    m.start(7, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-3, 0.0, 0.0), (2, 0.0, 0.0)], blocked=True)
    check("blocked progress: machine steps in", r[-1]["mode"] == "pull_out")
    m.start(8, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-30, 9.0, 0.0)])
    check("push-back while clear of the hole (free air): no trigger", r[-1]["mode"] == "policy")


def test_real_robot_rules():
    m = machine()
    m.start(9, ins(-20, 60), None, None, None, pose(-20))
    r = run(m, [(-10, 0.0, 60.0), (-0.5, 0.0, 60.0), (0.5, 0.0, 60.0)])
    check("incident case: 60 mm off axis -- machine steps in approaching the rim (0.5 mm above)",
          r[1]["mode"] == "pull_out" and r[0]["mode"] == "policy")
    m.start(13, ins(-20, 60), None, None, None, pose(-20))
    r = run(m, [(-10, 0.0, 60.0), (0.5, 0.0, 60.0)])
    check("tip below the rim outside the hole: TERMINATED OFF_AXIS at once, fail priced ~ -1",
          r[-1]["frame_state"] == wire.FS_TERMINATED and r[-1]["reason"] == wire.TR_OFF_AXIS
          and r[-1]["step"] == 2 and r[-1]["terms"]["fail"] < -0.9)
    m.start(14, ins(-20), None, None, None, pose(-20))
    r = run(m, [(5, 0.0, 2.5), (10, 0.0, 2.0)])
    check("2.5 mm off axis in the hole (within the 3 mm range): no OFF_AXIS stop",
          all(x["frame_state"] == wire.FS_POLICY for x in r))
    m.start(10, ins(-20), None, None, None, pose(-20))
    m.force[0] = 0.0
    r1 = m.tick(DT, ins(5), None, None, None, pose(5), dj7=250.0)
    r2 = m.tick(DT, ins(6), None, None, None, pose(6), dj7=350.0)
    check("left j7 +250 mA in the hole: no trigger; +350 mA: machine steps in",
          r1["mode"] == "policy" and r2["mode"] == "pull_out")
    m.start(11, ins(-20), None, None, None, pose(-20))
    r = m.tick(DT, ins(5), None, None, None, pose(5), hole_shift=0.007)
    check("hole pushed 7 mm sideways: TERMINATED SAFETY", r["frame_state"] == wire.FS_TERMINATED and r["reason"] == wire.TR_SAFETY)
    m.start(12, ins(-20), None, None, None, pose(-20))
    run(m, [(-3, 0.0, 0.0), (5, 7.0, 0.0)])                   # -> pull_out
    r = m.tick(DT, ins(3), None, None, None, pose(3), hole_shift=0.007)
    check("hole pushed during the machine's pull-out: TERMINATED SAFETY too",
          r["frame_state"] == wire.FS_TERMINATED and r["reason"] == wire.TR_SAFETY)


def test_machine_deltas():
    m = machine()
    d = m.pull_out_delta(np.array([0.0, 0.0, 1.0]))
    check("pull-out step: 6.67 mm up the axis, no rotation", np.allclose(d, [0, 0, m.cfg.max_trans, 0, 0, 0]))
    A, B = pose(0), T_from([0.5, 0.05, 0.9], np.eye(3))
    d = m.toward_delta(A, B)
    check("trace-back step capped per axis at 6.67 mm", np.isclose(d[1], m.cfg.max_trans) and np.allclose(d[[0, 2, 3, 4, 5]], 0))


if __name__ == "__main__":
    test_clean_success()
    test_intervention_cycle()
    test_intervention_limit()
    test_other_terminations()
    test_real_robot_rules()
    test_machine_deltas()
    print("\nall passed" if not failures else f"\n{len(failures)} failed: {failures}")
    sys.exit(1 if failures else 0)
