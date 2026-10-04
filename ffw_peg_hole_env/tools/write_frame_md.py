#!/usr/bin/env python3
"""Regenerate FRAME.md from wire.py's field tables (tests check they match).

  ../ffw_collision_checker/scripts/.venv/bin/python tools/write_frame_md.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ffw_peg_hole_env import wire  # noqa: E402

INTRO = f"""# Frame (msg type {wire.MSG_FRAME}) — real-robot peg-hole server

One `Frame` per 15 Hz tick on PUB port base+0 (`peg_hole_serl_server.py`). Little-endian, {wire.FRAME_BYTES} bytes:

| bytes | content |
|---|---|
| 0:12 | header: int32 type = {wire.MSG_FRAME}, float64 timestamp (server `time.time()`) |
| 12:{12 + 8 * wire.OBS_N} | **obs**: {wire.OBS_N} float64 — what a deployed actor can measure |
| {12 + 8 * wire.OBS_N}:{12 + 8 * (wire.OBS_N + wire.PRIV_N)} | **priv**: {wire.PRIV_N} float64 — everything the server derives (critic inputs, logging) |
| {12 + 8 * (wire.OBS_N + wire.PRIV_N)}:{wire.FRAME_BYTES} | **tag**: uint32 episode_id, uint32 step, uint8 frame_state, uint8 reason, float64 reward, 6 float64 action |

`wire.decode_frame(data)` → `(obs, priv, tag, ts)`: dicts of named numpy arrays (size-1 fields
as floats), each with `"vector"` = the whole flat block; `obs` also has `joint_pos`,
`joint_vel`, `joint_effort` (25 each). `wire.py` needs only struct + numpy.

The server packs everything; the client chooses what the actor and the critic see
(asymmetric actor-critic). obs and priv travel in the same message, so they can never be
paired with the wrong tick.

Conventions: base_link; poses (x, y, z, roll, pitch, yaw) with R = Rx·Ry·Rz (the Obs /
`ControlCmdDelta` convention); m, rad, s, N, A unless stated. Joints in
`wire.JOINT_STATE_NAMES` order. **No NaN anywhere**: a field that does not apply reads 0 (episode
values outside an episode); `*_valid` / `image_ok` flags mark where 0 would be ambiguous, ages are
capped at {wire.NO_NAN_CAP:g} s (= none). Type-12 frames (the 2026-10-04 demos, NaN for "n/a") still
decode: `decode_frame` upgrades them to this layout. Velocities are finite differences between consecutive frames, angular
velocity as a vector (not rpy rates). Limit margins: the IK EE pose in the arm's
`peg_hole_frame_<l|r>` (static TF from `peg_hole_safe_setup.py`) against the
`peg_hole_safe_real_<l|r>` box in `ffw_collision_checker/config/limit_profiles.txt`;
`NO_LIMIT_DIFF` = {wire.NO_LIMIT_DIFF} on an arm whose frame is not on TF (see
`priv.limit_frame_ok`).

Tag `frame_state`: {", ".join(f"{i} {n}" for i, n in enumerate(wire.FS_NAMES))}. Store POLICY and
INTERVENTION frames, TERMINATED ends the episode, skip RESET / IDLE / FAULT. `reason` (on
TERMINATED): {", ".join(f"{i} {n}" for i, n in enumerate(wire.TR_NAMES) if i)}. `action` = the
right-arm delta actually applied this tick (the policy's after clipping, or the machine's on
INTERVENTION frames); `priv.policy_delta` is what the policy sent (`policy_delta_valid`).

"""


def text():
    return INTRO + wire.layout_markdown() + "\n"


if __name__ == "__main__":
    (ROOT / "FRAME.md").write_text(text())
    print(f"wrote {ROOT / 'FRAME.md'}")
