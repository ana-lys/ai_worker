# Peg-in-hole HIL-SERL interface (real robot)

The contract between the robot-side server (`peg_hole_serl_server.py`, ROS workstation) and a
HIL-SERL client. Version: **Frame type 13**, obs 123 + priv 195 float64, 2622 bytes
(`wire.MSG_FRAME`, `wire.OBS_N`, `wire.PRIV_N`, `wire.FRAME_BYTES`). 2026-10-04.

`wire.py` is the reference implementation and the only file a client needs (struct + numpy +
pyzmq for the socket). `FRAME.md` lists every obs / priv field.

## Sockets

ZMQ, both sockets bound by the server; ports = base + offset (server `--port-base`, default 7601).

| port | server socket | client socket | traffic |
|---|---|---|---|
| base + 0 | PUB (bind) | SUB (connect, subscribe "") | server → client: one Frame per 15 Hz tick |
| base + 1 | SUB (bind) | PUB (connect) | client → server: `ControlCmdDelta`, `EnvCmd` |

Every message starts with a 12-byte little-endian header: `int32 type, float64 timestamp`
(sender's `time.time()`). Wrong size or type → ignored (counted as bad).

## Server → client: Frame (type 13)

```
header (12) | obs: 123 float64 | priv: 195 float64 | tag: uint32 episode_id, uint32 step,
            |                  |                   |      uint8 frame_state, uint8 reason,
            |                  |                   |      float64 reward, 6 float64 action
```

- **obs** — what a deployed actor can measure: 25 joints × `[pos, vel, effort]` interleaved
  (effort in A on the arm joints), right and left IK EE pose + velocity (base_link,
  x y z roll pitch yaw with R = Rx·Ry·Rz), and per arm the (lo, hi) margins on 6 axes to its
  safety box. Feed the actor obs (+ images).
- **priv** — everything the server derives (critic inputs / logging): reward + terms + episode
  return, the push-back force estimate (N), depth / lateral / tilt of the peg against the
  calibrated hole axis, hole pose, commanded EE pose, the policy's raw delta, env flags
  (mode, interventions, can_touch, success, …), image ages, the gateway's full Obs.
  **No NaN anywhere** (a learner that drops NaN transitions is safe): a field that does not
  apply reads 0, with `force_ref_valid`, `policy_delta_valid`, `image_ok` flags where 0 would be
  ambiguous; `delta_age` / `image_age` are capped at 10 s (= none).
- **tag** — the label of this frame; it travels inside the same message as obs/priv, so a
  frame can never be paired with the wrong label.

`wire.decode_frame(bytes)` → `(obs, priv, tag, ts)` as named numpy arrays (`"vector"` = the
whole flat block; `obs["joint_pos"|"joint_vel"|"joint_effort"]` split out).

**With images** (server started with `--images`): the message is multipart
`[Frame, right RGB, left RGB]`, each image 128×128×3 uint8 raw RGB (right wrist = peg, left
wrist = hole; fixed ROIs cropped from the D405s). An empty image part = no image this tick.
Always receive with `recv_multipart()` and decode with `wire.decode_frame_parts(parts)` →
`(obs, priv, tag, ts, {"right": img|None, "left": img|None})`; it accepts a bare Frame too.

### frame_state

| value | name | meaning | store? |
|---|---|---|---|
| 0 | IDLE | no episode (waiting for RESET, or PAUSEd) | no |
| 1 | POLICY | the client's delta drove this tick | yes |
| 2 | INTERVENTION | the machine drove it (pull the peg out up the axis, trace back to the last pose near the hole top) — the episode continues afterwards | yes, as intervention |
| 3 | TERMINATED | last frame of the episode; `reason` says why | yes, done |
| 4 | RESET | between episodes (pull out, move the hole, seat it, peg to its start) | no |
| 5 | FAULT | the reset failed — robot holds; needs a person, then `EnvCmd RESET` | no |

`reason` (TERMINATED only): 1 SUCCESS (tip ≥ 34 mm in, ≤ 2 mm off the axis), 2 JAM
(push-back > 10 N), 3 RIM (> 8 N for 2 ticks within 2 mm of the rim), 4 BLOCKED (a 4th
intervention was needed while progress was blocked), 5 TIMEOUT (20 s), 6 SAFETY (gear guard, hole pushed > 6 mm, hard edge, left j7), 7 INTERVENTIONS
(a 4th was needed), 8 ABORT. (9 OFF_AXIS reserved, not sent.)

`action` (tag) = the right-arm delta actually applied this tick, metres / radians: the client's
after clipping on POLICY frames, the machine's on INTERVENTION frames. `priv.policy_delta` is
what the client sent.

`reward` per tick: success bonus 1 + 0.5·(1 − F_peak/10 N) on SUCCESS; potential-based
progress shaping (≤ 0.3 per episode); −0.01·clip(F/10 N) force penalty every tick; on a JAM /
RIM / other failure −0.5 − 0.5·(share of the 34 mm not reached); none on TIMEOUT. Terms are in priv
(`r_success`, `r_shape`, `r_force`, `r_fail`, `return`).

## Client → server

**ControlCmdDelta (type 6)** — 14 float64: right `(dx, dy, dz, droll, dpitch, dyaw, grip)`,
then left (same). Only the right translation/rotation is used; grip and the left arm are
ignored.
- applied to the right arm's commanded IK EE pose: translation in base_link, rotation
  `R_new = R_delta · R`, R_delta from intrinsic X-Y-Z angles (the obs rpy convention);
- clipped per axis to **6.67 mm / 2° per tick** — that is action ±1. A normalized policy
  action `a ∈ [-1, 1]^6` is sent as `a * (0.00667, 0.00667, 0.00667, 0.0349, 0.0349, 0.0349)`;
- then clamped to a box around the hole and, farther than 2 cm from the hole axis, kept 5 mm
  above the rim (the `clamped` flag in priv says when);
- the **newest** delta received before a tick is applied (not summed); none for 0.2 s → hold
  (zero delta);
- used on POLICY ticks only (ignored during RESET / INTERVENTION / IDLE).

**EnvCmd (type 8)** — `uint8 cmd, int64 seed, uint32 episode_id, 8 float64 reserved`:
1 RESET — start the automatic episode loop; the next episode gets `episode_id` (0 = keep
counting); 2 PAUSE — stop after the current episode (IDLE); 3 RESUME; 4 ABORT — end the
episode now (TERMINATED ABORT, then IDLE). The real server ignores `seed` (hole and peg start
are sampled server-side) and PING (5).

## Episode loop and timing

```
client: EnvCmd RESET (once)
server: RESET frames … then POLICY step 0 (the observation after the reset, action 0)
loop at 15 Hz:
  client: on each POLICY frame k, send ControlCmdDelta for the next tick
  server: applies it during tick k+1, publishes frame k+1 (tag.action = what was applied,
          tag.reward, frame_state)
  … INTERVENTION frames when the machine steps in (keep stepping; your deltas are ignored)
  … TERMINATED frame → server resets by itself (RESET frames) → next episode's step 0
```

A transition is `(obs of frame k-1, tag.action of frame k, tag.reward of frame k, obs of
frame k, done = frame k is TERMINATED)`. Interventions are machine-driven (labelled
INTERVENTION), not human.

The reset (about 10 s, not recorded): peg pulled straight up out of the hole, hole arm moved
to a new random pose (±4 cm across, 0…−2 cm along, ±3° tilt), the hole block seated in the
gripper by an 8 N press beside the hole, peg moved to a random start 2–4 cm above the rim,
±5 cm across, ±5° tilt. An episode then takes ~7 s for a scripted expert.

## Expert demos

Recorded with `peg_hole_demo_recorder.py`: 100 episodes, every one SUCCESS on the first try,
peak force median 1.1 N. One npz per episode (`pose_KKK.npz`):

| key | shape | content |
|---|---|---|
| `frames` | (N, 2590) uint8 | raw type-12 Frames, step 0 … TERMINATED — `wire.decode_frame(row.tobytes())` reads them and returns the type-13 layout (NaN replaced, flags derived) |
| `img_right`, `img_left` | (N, 128, 128, 3) uint8 | the wrist images of each frame (RGB) |
| `img_ok` | (N, 2) bool | image present (right, left) |
| `meta` | JSON string | reason, interventions, offset, start pose, … |

`export_demos.py` turns a demo directory into one pkl in the hil-serl demo layout
(`{"schema", "transitions"}`, observations `state` / `priv` / `image_right` / `image_left`,
actions normalized to [-1, 1] as above).

## Versioning

Any layout change bumps `MSG_FRAME` and changes `FRAME_BYTES`; 13 = 12 without NaN (+4 flag / age
fields; `decode_frame` still reads 12, upgraded); a client built against another
layout gets a size / type error from `decode_frame`, never silently shifted fields. Types 9
(EnvStatus) and 10 (Images) in `wire.py` belong to the MuJoCo sim server only; type 11 (the
first Frame) is retired.

## Known limitations

- The left (hole) arm sometimes drifts up to ~10–16 mm early in an episode while its goal is
  fixed (the IK couples the arms when the right arm moves fast); `priv.hole_pose` shows it,
  and the hole axis used for every rule is measured live.
- The force is estimated from motor currents (no F/T sensor): ±5 N noise in free air while
  moving; contact rules use 2-tick holds.
