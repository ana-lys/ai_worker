# Notice to the HIL-SERL client — server changes since the first kit (2026-10-04)

The first kit spoke **Frame type 12**. The server now speaks **type 14**. Everything below is in
`SERL_INTERFACE.md` / `FRAME.md`; this is the short list of what you must change.

## 1. Decode type 14 — and stop worrying about NaN

- `wire.MSG_FRAME = 14`, `FRAME_BYTES = 2646` (obs 123 + **priv 198** + tag). Use the new `wire.py`;
  an old decoder fails loudly on the size, it cannot mis-read fields.
- **No NaN anywhere** (the learner's `check_nan_in_transition` drops NaN transitions — this was
  every step 0 and every dropped camera frame). Fields that don't apply read 0; flags say where 0
  is ambiguous: `force_ref_valid`, `policy_delta_valid`, `image_ok` (2); `image_age` (2, replaces
  `image_t`) and `delta_age` are capped at 10 s = none. **Do not patch NaN on the client.**
- New priv fields: `reset_kind` (0 new pair / 1 retry), `retry_count`, `parent_episode`.
- `wire.decode_frame` still reads type 12 and 13 (e.g. the 100 recorded demos) and returns the
  type-14 layout.

## 2. Every `env.reset()` sends `EnvCmd RESET` with a mode

The server no longer starts the next episode by itself. After TERMINATED it **pulls the peg out
straight away** (RESET frames — the peg may be loaded against the hole), then **waits (IDLE)**.

```python
pub.send(wire.encode_env_cmd(wire.RESET, params=(float(mode), 0, 0, 0, 0, 0, 0, 0)))
# then wait for the frame with frame_state == POLICY and step == 0: that is the reset() obs
```

| mode (`params[0]`) | next episode |
|---|---|
| `wire.RESET_AUTO` (0) | after JAM / RIM / BIND / OFF_AXIS / BLOCKED: a **retry** of the same pair from the failed episode's **last good state** (last policy pose with the tip ≥ 3 mm above the rim), ≤ 3 retries per pair; otherwise a new random pair |
| `wire.RESET_RANDOM` (1) | always a new random pair — use for evaluation, or a share of training resets as a curriculum mix |
| `wire.RESET_RETRY` (2) | retry the last episode (not after SAFETY / ABORT) |

## 3. Failures end the episode (no more mid-episode machine interventions)

Binding no longer triggers an in-episode pull-out (INTERVENTION frames). It **ends the episode
with the negative fail reward**, and the retry is a **new** episode that can end positive:

| reason | | done | RESET AUTO next |
|---|---|---|---|
| SUCCESS 1 | positive | terminated | new pair |
| JAM 2, RIM 3, **BIND 10** (new), **OFF_AXIS 9** (now sent), BLOCKED 4 | negative | terminated | retry |
| SAFETY 6, ABORT 8 | negative | terminated | new pair |
| TIMEOUT 5 | none | **truncated** (keep bootstrapping) | new pair |

INTERVENTION frames (2) only appear if the server runs `--failure-mode intervene`; if you ever
see them, store them with `is_intervention=False` (machine, not expert).

## 4. Reply to every POLICY frame immediately — lockstep

A POLICY tick now waits up to **40 ms** for your reply to the previous frame and applies it in the
same tick, so **frame k+1's `tag.action` is the action you chose from frame k** — the transition
pairing is exact. (Before this the reply always landed one tick late, so stored actions were
shifted by a frame.) Keep decode → policy → send under ~40 ms; a late reply is applied a tick late,
no reply → hold (`policy_delta_valid` 0).

Transition: `(obs[k-1], tag[k].action / ACTION_SCALE, tag[k].reward, obs[k],
terminated = TERMINATED and reason != TIMEOUT, truncated = reason == TIMEOUT)`.

## 5. Verified locally

`tests/test_server_local.py` runs the real server against a fake robot with a ZMQ client on
localhost: RESET AUTO → BIND (−0.95) → pull-out → IDLE → RESET AUTO → retry from the good state
(tip 4 mm above the rim, where the failure left it) → SUCCESS (+1.50) → new pair; the 3-retry cap;
RESET RANDOM; every frame type 14 and finite; 84/84 actions paired with the right frame. Not yet
run on the robot.

## Server flags (robot side, for reference)

`--failure-mode terminate|intervene` (default terminate), `--auto-reset` (old loop: reset without
waiting for RESET), `--max-retries 3`, `--images`.
