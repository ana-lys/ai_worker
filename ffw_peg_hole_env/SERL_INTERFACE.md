# Peg-in-hole HIL-SERL interface (real robot)

The contract between the robot-side server (`peg_hole_serl_server.py`, ROS workstation) and a
HIL-SERL client. Version: **Frame type 14**, obs 123 + priv 198 float64, 2646 bytes
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

## Server → client: Frame (type 14)

```
header (12) | obs: 123 float64 | priv: 198 float64 | tag: uint32 episode_id, uint32 step,
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
  (mode, can_touch, success, …), how the episode started (`reset_kind` 0 new pair / 1 retry,
  `retry_count`, `parent_episode`), image ages, the gateway's full Obs.
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
| 2 | INTERVENTION | only with `--failure-mode intervene`: the machine drove it — pulled the peg out of the bad state, then the scripted expert to the goal; same episode, the policy does not get control back | yes, **`is_intervention=True`** (expert data: also into the offline / demo buffer) |
| 3 | TERMINATED | last frame of the episode; `reason` says why | yes, done |
| 4 | RESET | between episodes: the pull-out after TERMINATED, and the reset after `EnvCmd RESET` | no |
| 5 | FAULT | the reset failed — robot holds; needs a person, then `EnvCmd RESET` | no |

`reason` (TERMINATED only):

| value | name | when | negative fail reward | RESET AUTO next |
|---|---|---|---|---|
| 1 | SUCCESS | tip ≥ 34 mm in, ≤ 2 mm off the axis | — (success bonus) | new pair |
| 2 | JAM | push-back > 10 N | yes | retry |
| 3 | RIM | > 8 N for 2 ticks within 2 mm of the rim | yes | retry |
| 10 | BIND | push-back > 7 N for 2 ticks (or the hole arm loaded > 300 mA) | yes | retry |
| 9 | OFF_AXIS | tip below the rim > 2 cm off the axis (heading down beside the block) | yes | retry |
| 4 | BLOCKED | the command descends, the peg does not | yes | retry |
| 5 | TIMEOUT | 20 s | no | new pair |
| 6 | SAFETY | gear guard, hole pushed > 6 mm, hard edge (25 N), left j7 | yes | new pair |
| 8 | ABORT | `EnvCmd ABORT` | yes | new pair |
| 7 | INTERVENTIONS | retired (not sent) | | |

Map TIMEOUT to **truncated** (not terminated): the time ran out, the state is not bad, keep
bootstrapping. Every other reason is a true terminal.

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
- **lockstep**: a POLICY tick waits up to 40 ms for the client's reply to the previous frame and
  applies it in that tick, so frame k+1's `tag.action` is exactly the action chosen from frame k.
  Reply to every POLICY frame right away (decode → policy → send within ~40 ms); a later reply is
  applied a tick late, none → hold (zero delta, `policy_delta_valid` 0). If several arrive, the
  newest wins (not summed);
- used on POLICY ticks only (ignored during RESET / INTERVENTION / IDLE).

**EnvCmd (type 8)** — `uint8 cmd, int64 seed, uint32 episode_id, 8 float64 params`:
- 1 **RESET** — start the next episode; `params[0]` = the start (`wire.RESET_*`):

  | params[0] | mode | the next episode starts |
  |---|---|---|
  | 0 | AUTO (default) | after a JAM / RIM / BIND / OFF_AXIS / BLOCKED failure: a **retry** — same hole (re-seated), peg at the failed episode's **last good state** (the last policy pose with the tip ≥ 3 mm above the rim: the closest pose outside the hole); at most 3 retries per pair. Otherwise: a **new random pair** |
  | 1 | RANDOM | always a new random pair (evaluation, or the client's curriculum mix) |
  | 2 | RETRY | a retry of the last episode whatever its reason (except SAFETY / ABORT → new pair) |

  `episode_id` sets the next episode's id (0 = keep counting).
- 2 PAUSE — stop after the current episode (IDLE); 3 RESUME; 4 ABORT — end the episode now
  (TERMINATED ABORT). The real server ignores `seed` and PING (5).

## Episode loop and timing

```
env.reset():  client sends EnvCmd RESET(params[0] = mode)
              server: RESET frames (the reset) … then POLICY step 0 (action 0)
env.step():   client: on each POLICY frame k, send ControlCmdDelta for the next tick
              server: applies it during tick k+1, publishes frame k+1 (tag.action = what was
                      applied, tag.reward, frame_state)
              … TERMINATED frame k (reason, fail / success reward) = done
              server, at once and by itself: pulls the peg straight out of the hole (RESET
              frames — the peg may be loaded against the hole), then holds: IDLE frames
env.reset():  client sends the next EnvCmd RESET …
```

The **client decides when** the next episode starts; the **server decides how** (it knows the
hole, the good state, whether the hole was pushed) unless the client forces RANDOM / RETRY.
Everything about an episode — including how it started — is in its Frames; there is no other
message. `--auto-reset` (server) restores the old loop: after the pull-out it resets by itself
(mode = the last RESET's) without waiting.

A transition is `(obs of frame k-1, tag.action of frame k, tag.reward of frame k, obs of
frame k, done = frame k is TERMINATED and reason != TIMEOUT, truncated = reason == TIMEOUT)`.
A failure therefore ends its episode with the negative reward; the retry is a NEW episode
(`reset_kind` 1, `parent_episode` = the failed one) that starts at the good state and can end
with the success reward.

`--failure-mode intervene` (server) is a **SERL intervention** instead: on a trigger the policy's
last frame gets the fail penalty (the bad state), then the machine takes over for the rest of the
episode — pulls the peg straight out (`priv.mode` 2), then the scripted expert (the one that
recorded the 100 demos) aligns over the axis and inserts (`priv.mode` 6) — INTERVENTION frames
whose `tag.action` is the expert's applied delta; the episode ends SUCCESS (or the expert's own
failure reason) and gets 10 s extra before TIMEOUT. Store INTERVENTION transitions with
`is_intervention=True`, exactly like a human (SpaceMouse) intervention on the client side.
`terminate` (+ retry) teaches "don't go there"; `intervene` also shows how to get out and finish.

A new-pair reset (about 10 s, not recorded): hole arm moved to a new random pose (±4 cm across,
0…−2 cm along, ±3° tilt), the hole block seated in the gripper by an 8 N press beside the hole,
peg moved to a random start 2–4 cm above the rim, ±5 cm across, ±5° tilt. A retry reset: the
hole stays (re-seated), the peg goes to the good state. An episode takes ~7 s for a scripted
expert.

## Expert demos

Recorded with `peg_hole_demo_recorder.py`: 100 episodes, every one SUCCESS on the first try,
peak force median 1.1 N. One npz per episode (`pose_KKK.npz`):

| key | shape | content |
|---|---|---|
| `frames` | (N, 2590) uint8 | raw type-12 Frames, step 0 … TERMINATED — `wire.decode_frame(row.tobytes())` reads them and returns the type-14 layout (NaN replaced, flags derived, retry fields 0) |
| `img_right`, `img_left` | (N, 128, 128, 3) uint8 | the wrist images of each frame (RGB) |
| `img_ok` | (N, 2) bool | image present (right, left) |
| `meta` | JSON string | reason, interventions, offset, start pose, … |

`export_demos.py` turns a demo directory into one pkl in the hil-serl demo layout
(`{"schema", "transitions"}`, observations `state` / `priv` / `image_right` / `image_left`,
actions normalized to [-1, 1] as above).

## Versioning

Any layout change bumps `MSG_FRAME` and changes `FRAME_BYTES`. 14 = 13 + the three retry fields;
13 = 12 without NaN (+4 flag / age fields); `decode_frame` reads 12 and 13 too, upgraded to 14; a client built against another
layout gets a size / type error from `decode_frame`, never silently shifted fields. Types 9
(EnvStatus) and 10 (Images) in `wire.py` belong to the MuJoCo sim server only; type 11 (the
first Frame) is retired.

## Known limitations

- The left (hole) arm sometimes drifts up to ~10–16 mm early in an episode while its goal is
  fixed (the IK couples the arms when the right arm moves fast); `priv.hole_pose` shows it,
  and the hole axis used for every rule is measured live.
- The force is estimated from motor currents (no F/T sensor): ±5 N noise in free air while
  moving; contact rules use 2-tick holds.
