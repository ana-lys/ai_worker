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
from ffw_peg_hole_env.episode import EpisodeConfig, EpisodeMachine  # noqa: E402
from ffw_peg_hole_env.geometry import T_from  # noqa: E402

failures = []
DT = 1 / 15


def check(name, cond):
    print(f"{'ok  ' if cond else 'FAIL'} {name}")
    if not cond:
        failures.append(name)


def machine(failure_mode="intervene"):
    m = EpisodeMachine(EpisodeConfig(failure_mode=failure_mode))
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
    """--failure-mode intervene = a SERL intervention: penalty on the bad state, then the machine
    (pull out, then the expert) drives to the goal; the policy does not get control back."""
    m = machine()
    m.start(2, ins(-20), None, None, None, pose(-20))
    r = run(m, [(d, 2.0, 0.0) for d in (-15, -10, -6, -3, 0, 4)] + [(6, 7.5, 0.0), (7, 2.0, 0.0), (8, 6.5, 0.0), (8.5, 6.5, 0.0)])
    check("one 7.5 N tick, or 6.5 N held: no intervention (normal chamfer entry)", all(x["mode"] == "policy" for x in r))
    r = run(m, [(9, 7.5, 0.0), (10, 7.5, 0.0)])                                     # binds at 10 mm
    check("binding at 7.5 N for 2 ticks: frame POLICY (the policy drove it) with the fail penalty, next pull_out",
          r[-1]["frame_state"] == wire.FS_POLICY and r[-1]["mode"] == "pull_out" and r[-1]["interventions"] == 1
          and r[-1]["terms"]["fail"] < -0.4 and r[-1]["reward"] < -0.4)
    r = run(m, [(6, 12.0, 0.0)] + [(d, 0.0, 0.0) for d in (2, -2, -6)])     # 12 N while pulling out (2026-10-04 live)
    check("12 N during the machine's pull-out: no further penalty, episode goes on",
          r[0]["terms"]["fail"] == 0.0 and r[0]["frame_state"] == wire.FS_INTERVENTION and r[0]["reward"] > -0.1)
    check("pulling out: INTERVENTION frames, the expert takes over once the tip is 5 mm clear",
          all(x["frame_state"] == wire.FS_INTERVENTION for x in r) and r[-1]["mode"] == "expert"
          and [x["mode"] for x in r[:-1]] == ["pull_out"] * 3)
    r = run(m, [(d, 1.0, 0.0) for d in (-5, -3, 0, 10, 20, 30)])
    check("the expert inserting: INTERVENTION frames, mode stays expert", all(x["frame_state"] == wire.FS_INTERVENTION
          and x["mode"] == "expert" for x in r))
    r = run(m, [(34.5, 1.0, 0.0)])
    check("the expert reaches the goal: TERMINATED SUCCESS, the success reward, same episode",
          r[-1]["frame_state"] == wire.FS_TERMINATED and r[-1]["reason"] == wire.TR_SUCCESS
          and r[-1]["terms"]["success"] > 0.9 and m.episode_id == 2)


def test_intervention_expert_fails():
    m = machine()
    m.start(3, ins(-20), None, None, None, pose(-20))
    run(m, [(-3, 0.0, 0.0), (5, 7.5, 0.0), (5, 7.5, 0.0)])                   # trigger -> pull_out
    run(m, [(d, 0.0, 0.0) for d in (2, -2, -6)])                             # -> expert
    r = run(m, [(4, 7.5, 0.0), (5, 7.5, 0.0)])
    check("the expert binding too: TERMINATED BIND with a fail penalty (no second intervention)",
          r[-1]["frame_state"] == wire.FS_TERMINATED and r[-1]["reason"] == wire.TR_BIND and r[-1]["terms"]["fail"] < -0.4
          and m.interventions == 1)
    m = machine()
    m.start(4, ins(-20), None, None, None, pose(-20))
    run(m, [(-3, 0.0, 0.0), (5, 7.5, 0.0), (5, 7.5, 0.0)])
    steps = int(round((m.cfg.timeout_s + 2) / DT))
    r = run(m, [(-6, 0.0, 0.0)] * steps)
    check(f"an intervention adds {m.cfg.intervene_time:.0f} s to the timeout (no TIMEOUT at {m.cfg.timeout_s + 2:.0f} s)",
          all(x["reason"] != wire.TR_TIMEOUT for x in r))


def test_rim_band():
    m = machine()
    m.start(6, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-3, 0.0, 0.0), (0.5, 6.5, 0.0), (1.0, 6.5, 0.0), (1.5, 6.5, 0.0)])
    check("light edge contact at the rim (6.5 N held): no intervention, no end", all(x["mode"] == "policy" for x in r))
    r = run(m, [(1.0, 8.5, 0.0), (1.5, 8.5, 0.0)])
    check("8.5 N for 2 ticks within 2 mm of the rim: TERMINATED RIM",
          r[-1]["frame_state"] == wire.FS_TERMINATED and r[-1]["reason"] == wire.TR_RIM)
    m = machine()
    m.start(7, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-3, 0.0, 0.0), (3.0, 8.5, 0.0), (3.5, 8.5, 0.0)])
    check("8.5 N for 2 ticks 3 mm in (past the rim band): machine intervenes, episode goes on",
          r[-1]["frame_state"] == wire.FS_POLICY and r[-1]["mode"] == "pull_out")


def test_terminate_mode():
    """--failure-mode terminate (the default): a trigger ends the episode, priced like a failure."""
    check("default failure mode is terminate", EpisodeConfig().failure_mode == "terminate")
    m = machine("terminate")
    m.start(8, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-3, 0.0, 0.0), (5, 7.5, 0.0), (6, 7.5, 0.0)])
    check("binding 7.5 N x2: TERMINATED BIND with a fail penalty, no intervention",
          r[-1]["frame_state"] == wire.FS_TERMINATED and r[-1]["reason"] == wire.TR_BIND
          and r[-1]["terms"]["fail"] < -0.5 and r[-1]["interventions"] == 0 and r[-1]["mode"] == "reset")
    m = machine("terminate")
    m.start(9, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-3, 0.0, 30.0), (0.5, 0.0, 30.0)])
    check("tip below the rim 3 cm off the axis: TERMINATED OFF_AXIS", r[-1]["reason"] == wire.TR_OFF_AXIS)
    m = machine("terminate")
    m.start(10, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-3, 0.0, 0.0), (2, 0.0, 0.0)], blocked=True)
    check("blocked progress: TERMINATED BLOCKED", r[-1]["reason"] == wire.TR_BLOCKED)


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
    check("4 mm off the axis just above the rim: free (inside the zone)", r[-1]["mode"] == "policy")
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
    check("incident case: 60 mm off axis -- free above the rim, machine halts it once the tip goes below",
          r[0]["mode"] == "policy" and r[1]["mode"] == "policy" and r[2]["mode"] == "pull_out"
          and r[2]["frame_state"] == wire.FS_POLICY)
    m.start(13, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-1, 0.0, 15.0), (0.5, 2.0, 15.0)])
    check("15 mm off axis at the rim (on the block top, inside the 2 cm zone): free interaction",
          all(x["mode"] == "policy" for x in r))
    m.start(14, ins(-20), None, None, None, pose(-20))
    r = run(m, [(5, 0.0, 2.5), (10, 0.0, 2.0)])
    check("2.5 mm off axis in the hole: no zone trigger", all(x["frame_state"] == wire.FS_POLICY for x in r))
    m.start(15, ins(-20), None, None, None, pose(-20))
    r = run(m, [(-1, 0.0, 25.0), (0.5, 0.0, 25.0)])
    check("25 mm off axis going below the rim: machine steps in", r[-1]["mode"] == "pull_out")
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
    check("pull-out step: 2 mm up the axis, no rotation", np.allclose(d, [0, 0, m.cfg.pull_out_step, 0, 0, 0]))
    A, B = pose(0), T_from([0.5, 0.05, 0.9], np.eye(3))
    d = m.toward_delta(A, B)
    check("trace-back step capped per axis at 6.67 mm", np.isclose(d[1], m.cfg.max_trans) and np.allclose(d[[0, 2, 3, 4, 5]], 0))


if __name__ == "__main__":
    test_clean_success()
    test_intervention_cycle()
    test_intervention_expert_fails()
    test_rim_band()
    test_terminate_mode()
    test_other_terminations()
    test_real_robot_rules()
    test_machine_deltas()
    print("\nall passed" if not failures else f"\n{len(failures)} failed: {failures}")
    sys.exit(1 if failures else 0)
