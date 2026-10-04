#!/usr/bin/env python3
"""Unit checks (no ZMQ, no robot): wire round-trips, bad-frame rejection, seed
reproducibility, reset failure handling.

  ../ffw_collision_checker/scripts/.venv/bin/python tests/test_wire_and_task.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import MujocoBackend, PegHoleConfig, PegHoleTask, wire  # noqa: E402
from ffw_peg_hole_env.geometry import apply_delta, delta_between  # noqa: E402
from ffw_peg_hole_env.server import compose  # noqa: E402

failures = []


def check(name, cond):
    print(f"{'ok  ' if cond else 'FAIL'} {name}")
    if not cond:
        failures.append(name)


def test_wire():
    r7, l7 = (0.01, -0.02, 0.003, 0.01, -0.02, 0.03, 1.0), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5)
    r, l, ts = wire.decode_delta(wire.encode_delta(r7, l7, ts=123.5))
    check("ControlCmdDelta round-trip", r == r7 and l == l7 and ts == 123.5)
    c, _ = wire.decode_env_cmd(wire.encode_env_cmd(wire.RESET, seed=-7, episode_id=42, params=tuple(range(8))))
    check("EnvCmd round-trip", c["cmd"] == wire.RESET and c["seed"] == -7 and c["episode_id"] == 42
          and c["params"] == tuple(float(v) for v in range(8)))
    st = {"state": wire.RUNNING, "backend": wire.BACKEND_SIM, "reset_ok": 1, "success": 0, "terminated": 0,
          "truncated": 1, "episode_id": 3, "step": 17, "deltas_received": 17, "reward": 0.0, "depth": -0.004,
          "lateral": 0.0012, "contact_force": 1.5}
    s2, _ = wire.decode_status(wire.encode_status(st))
    check("EnvStatus round-trip", s2 == st)
    a = np.random.default_rng(0).integers(0, 255, (128, 128, 3), dtype=np.uint8)
    b = a[::-1].copy()
    im, _ = wire.decode_images(wire.encode_images(5, 9, a, b))
    check("Images round-trip", im["episode_id"] == 5 and im["step"] == 9
          and np.array_equal(im["right"], a) and np.array_equal(im["left"], b))
    for name, bad in (("short frame", wire.encode_delta(r7, l7)[:-8]),
                      ("wrong type", wire.encode_env_cmd(wire.PING)[:12] + wire.encode_delta(r7, l7)[12:])):
        try:
            wire.decode_delta(bad)
            check(f"rejects {name}", False)
        except ValueError:
            check(f"rejects {name}", True)


def test_obs_layout():
    """decode_obs (standalone) against the gateway's own encode_obs, field by field."""
    gw = wire.gw
    if gw is None:
        check("gateway protocol importable for the Obs cross-check", False)
        return
    rng = np.random.default_rng(2)
    o = gw.Obs()
    o.joint_pos, o.joint_vel, o.joint_effort = (list(rng.normal(size=25)) for _ in range(3))
    o.ee = (tuple(rng.normal(size=6)), tuple(rng.normal(size=6)))
    o.gripper = (0.25, 0.75)
    o.base_marker = tuple(rng.normal(size=6))
    o.ee_marker = (tuple(rng.normal(size=6)), tuple(rng.normal(size=6)))
    o.limit_diff = (tuple(rng.normal(size=12)), tuple(rng.normal(size=12)))
    d, _ = wire.decode_obs(gw.encode_obs(o, ts=1.0))
    ok = (np.allclose(d["joint_pos"], o.joint_pos) and np.allclose(d["joint_vel"], o.joint_vel)
          and np.allclose(d["joint_effort"], o.joint_effort)
          and np.allclose(d["ee_right"], o.ee[0]) and np.allclose(d["ee_left"], o.ee[1])
          and d["grip_right"] == 0.25 and d["grip_left"] == 0.75
          and np.allclose(d["base_marker"], o.base_marker)
          and np.allclose(d["ee_right_marker"], o.ee_marker[0]) and np.allclose(d["ee_left_marker"], o.ee_marker[1])
          and np.allclose(d["limit_diff_right"], o.limit_diff[0]) and np.allclose(d["limit_diff_left"], o.limit_diff[1]))
    check("standalone decode_obs == gateway encode_obs, all 131 fields", ok)
    check("Obs frame is 1060 bytes", len(gw.encode_obs(o)) == 1060)
    rng = np.random.default_rng(3)
    obs, priv = rng.normal(size=wire.OBS_N), rng.normal(size=wire.PRIV_N)
    act = (0.001, -0.002, 0.003, 0.01, -0.02, 0.03)
    f = wire.encode_frame(obs, priv, 12, 34, wire.FS_INTERVENTION, wire.TR_NONE, -0.125, act, ts=7.25)
    fo, fp, tag, ts = wire.decode_frame(f)
    check(f"Frame is {wire.FRAME_BYTES} bytes, type 12, timestamp kept",
          len(f) == wire.FRAME_BYTES and wire.msg_type(f) == wire.MSG_FRAME == 12 and ts == 7.25)
    check("Frame obs / priv vectors round-trip", np.array_equal(fo["vector"], obs) and np.array_equal(fp["vector"], priv))
    check("obs names tile the vector in order",
          np.array_equal(np.concatenate([np.atleast_1d(fo[k]) for k, _, _ in wire.OBS_FIELDS]), obs))
    check("priv names tile the vector in order",
          np.array_equal(np.concatenate([np.atleast_1d(fp[k]) for k, _, _ in wire.PRIV_FIELDS]), priv))
    check("joints interleaved: joint j = obs[3j:3j+3]",
          np.array_equal(fo["joint_pos"], obs[0:75:3]) and np.array_equal(fo["joint_effort"], obs[2:75:3]))
    check("obs is 123 = 75 joints + 24 EE + 24 limit", wire.OBS_N == 123)
    check("Frame tag round-trip", tag["episode_id"] == 12 and tag["step"] == 34 and tag["frame_state"] == wire.FS_INTERVENTION
          and tag["reason"] == wire.TR_NONE and tag["reward"] == -0.125 and np.allclose(tag["action"], act))
    g = wire.decode_gateway_obs(fp["gateway_obs"])
    check("priv gateway_obs named like decode_obs", set(g) == set(d) and np.array_equal(g["ee_right"], fp["gateway_obs"][75:81]))
    bad = False
    try:
        wire.encode_frame(obs[:-1], priv, 0, 0, wire.FS_IDLE)
    except ValueError:
        bad = True
    check("encode_frame rejects a wrong-size obs", bad)
    bad = False
    try:
        wire.decode_frame(f[:-8])
    except ValueError:
        bad = True
    check("decode_frame rejects a short frame", bad)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import write_frame_md
    md = Path(__file__).resolve().parents[1] / "FRAME.md"
    check("FRAME.md matches wire.py (tools/write_frame_md.py regenerates it)", md.exists() and md.read_text() == write_frame_md.text())


def test_observation():
    from scipy.spatial.transform import Rotation as Rot
    from ffw_peg_hole_env import observation as ob
    from ffw_peg_hole_env.geometry import T_from
    gw = wire.gw
    rng = np.random.default_rng(5)
    for _ in range(20):
        R = Rot.random(random_state=rng)
        w = R.as_quat()                                           # x y z w
        ok = np.allclose(ob.rpy(R.as_matrix()), gw.quat_to_rpy(w[3], w[0], w[1], w[2]))
        if not ok:
            break
    check("rpy() == the gateway's quat_to_rpy (Obs ee convention)", ok)
    R = Rot.from_euler("XYZ", [0.1, -0.2, 0.3]).as_matrix()
    check("rpy(): R = Rx Ry Rz", np.allclose(ob.rpy(R), [0.1, -0.2, 0.3]))
    T0 = T_from([0.1, 0.2, 0.3], np.eye(3))
    T1 = T_from([0.1, 0.2, 0.31], Rot.from_rotvec([0, 0, 0.02]).as_matrix())
    check("velocity(): finite difference in base_link", np.allclose(ob.velocity(T0, T1, 0.1), [0, 0, 0.1, 0, 0, 0.2]))
    check("velocity(): zeros without a previous pose", np.array_equal(ob.velocity(None, T1, 0.1), np.zeros(6)))
    Tf = T_from([1.0, 0.0, 0.0], Rot.from_euler("z", 90, degrees=True).as_matrix())
    Ts = Tf @ T_from([0.01, -0.02, 0.0], np.eye(3))
    lo, hi = np.array([-0.05, -0.05, -0.05, -0.1, -0.1, -0.1]), np.array([0.05, 0.0, 0.05, 0.1, 0.1, 0.1])
    m = ob.limit_margins(Ts, Tf, lo, hi)
    check("limit_margins: pose in the box frame, (lo, hi) interleaved",
          np.allclose(m[:6], [0.06, 0.04, 0.03, 0.02, 0.05, 0.05]) and np.allclose(m[6:], 0.1))
    check("limit_margins: no frame -> NO_LIMIT_DIFF", np.all(ob.limit_margins(Ts, None, lo, hi) == wire.NO_LIMIT_DIFF))
    prof = ob.load_profiles()
    check("load_profiles: peg_hole_safe_real_l / _r with their frames",
          prof.get("l", ("",))[0] == "peg_hole_frame_l" and prof.get("r", ("",))[0] == "peg_hole_frame_r"
          and all(np.all(prof[a][1] < prof[a][2]) for a in prof))
    j = ob.joints_interleaved(np.arange(25.0), 100 + np.arange(25.0), 200 + np.arange(25.0))
    check("joints_interleaved", np.array_equal(j[:6], [0, 100, 200, 1, 101, 201]))


def test_standalone_import():
    """wire.py must work with only struct + numpy (no gateway module on the path)."""
    import subprocess
    src = Path(wire.__file__)
    code = ("import importlib.util, sys; sys.path = [p for p in sys.path if 'ffw_zmqinterface' not in p];"
            "import builtins; real = builtins.__import__\n"
            "def imp(n, *a, **k):\n"
            "    if n.startswith('ffw_zmqinterface'): raise ImportError(n)\n"
            "    return real(n, *a, **k)\n"
            "builtins.__import__ = imp\n"
            f"spec = importlib.util.spec_from_file_location('wire', '{src}'); w = importlib.util.module_from_spec(spec); spec.loader.exec_module(w)\n"
            "assert w.gw is None\n"
            "print(w.decode_status(w.encode_status(dict(state=2, backend=0, reset_ok=1, success=0, terminated=0, truncated=0, episode_id=1, step=0, deltas_received=0, reward=0.0, depth=0.0, lateral=0.0, contact_force=0.0)))[0]['state'])")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    check("wire.py imports and works without the gateway module", out.returncode == 0 and out.stdout.strip() == "2")


def test_delta_math():
    rng = np.random.default_rng(1)
    T = np.eye(4)
    T[:3, 3] = [0.4, 0.2, 0.9]
    for _ in range(20):
        d = np.concatenate([rng.uniform(-0.01, 0.01, 3), rng.uniform(-0.05, 0.05, 3)])
        if not np.allclose(delta_between(T, apply_delta(T, d)), d, atol=1e-12):
            check("delta_between inverts apply_delta", False)
            return
    check("delta_between inverts apply_delta", True)
    d1, d2 = rng.uniform(-0.01, 0.01, 6), rng.uniform(-0.01, 0.01, 6)
    check("compose(d1, d2) == d1 then d2",
          np.allclose(apply_delta(T, compose(d1, d2)) @ np.eye(4), apply_delta(apply_delta(T, d1), d2), atol=1e-12))


def test_safety_box():
    from ffw_peg_hole_env.geometry import SafetyBox, T_from
    from scipy.spatial.transform import Rotation as Rot
    R_f = Rot.from_euler("xyz", [20, -35, 50], degrees=True).as_matrix()      # a frame not aligned to base
    box = SafetyBox(T_from([0.5, 0.1, 0.9], R_f), [-0.05, -0.05, -0.02], [0.05, 0.05, 0.10],
                    np.radians([10, 10, 10]), np.eye(3))
    inside = T_from([0.5, 0.1, 0.9] + R_f @ [0.01, -0.02, 0.03], np.eye(3))
    out, hit = box.clamp(inside)
    check("SafetyBox leaves an inside pose alone", not hit and np.allclose(out, inside))
    far = T_from([0.5, 0.1, 0.9] + R_f @ [0.20, 0.0, -0.30], Rot.from_rotvec(R_f @ [0, 0, np.radians(40)]).as_matrix())
    out, hit = box.clamp(far)
    p, dev = box.where(out)
    check("SafetyBox clamps position along the frame's own axes",
          hit and np.allclose(p, [0.05, 0.0, -0.02], atol=1e-9))
    check("SafetyBox clamps rotation about the frame axes", np.isclose(dev[2], np.radians(10)) and abs(dev[0]) < 1e-9)


def test_task():
    b = MujocoBackend(cameras=False, timestep=0.004)
    t = PegHoleTask(b)
    _, i1 = t.reset(seed=11)
    _, i2 = t.reset(seed=11)
    _, i3 = t.reset(seed=12)
    same = all(np.allclose(i1[k], i2[k]) for k in ("hole_offset", "peg_offset", "peg_tilt_deg"))
    check("same seed -> same reset sample", same and i1["peg_tip_above_rim"] == i2["peg_tip_above_rim"])
    check("different seed -> different sample", not np.allclose(i1["peg_offset"], i3["peg_offset"]))
    unreachable = PegHoleTask(b, PegHoleConfig(hole_nominal=None, hole_base_up=1.0, reset_tries=3))   # hole 1 m up: out of reach
    try:
        unreachable.reset(seed=0)
        check("unreachable reset raises (server -> FAULT)", False)
    except RuntimeError:
        check("unreachable reset raises (server -> FAULT)", True)


if __name__ == "__main__":
    test_wire()
    test_obs_layout()
    test_observation()
    test_standalone_import()
    test_delta_math()
    test_safety_box()
    test_task()
    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: {failures}'}")
    sys.exit(1 if failures else 0)
