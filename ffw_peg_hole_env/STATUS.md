# Peg-in-hole HIL-SERL on the real robot — status (2026-10-05)

Short version: the robot side is done and live-tested. 100 fresh expert demos are recorded, and
the client kit is shipped. The HIL-SERL client connects and ran its first live episode. What's left
is mainly on the client (its actor stalls and is slow on the real robot), plus some housekeeping.
Details: `SERL_INTERFACE.md` (the contract), `FRAME.md` (fields), `CLIENT_NOTICE.md` (client changes),
`REAL_ROBOT_README.md` (history and older decisions).

## What exists

### Server (`ffw_collision_checker/scripts/peg_hole_serl_server.py`)
- **Wire:** ZMQ PUB base+0 sends one **Frame (type 14)** per 15 Hz tick: obs 123 (actor), priv 198
  (critic / logging), tag. With `--images` it is a multipart `[Frame, right RGB, left RGB]`, the 128×128
  wrist ROIs. SUB base+1 takes `ControlCmdDelta` and `EnvCmd`. There is **no NaN** anywhere (flags +
  capped ages instead). Older types 12/13 still decode.
- **Lockstep:** a policy tick waits ≤ 40 ms for the client's reply to the previous frame, so frame
  k+1's `tag.action` is the action chosen from frame k.
- **Episode loop:** the client sends `EnvCmd RESET(params[0] = AUTO / RANDOM / RETRY)` per episode.
  After TERMINATED the server pulls the peg out by itself, then waits in IDLE. `--auto-reset` keeps
  the old loop.
- **Failures end the episode** (`--failure-mode terminate`, the default). The reasons, with
  `RESET AUTO` behaviour:
  - BIND (7 N held 2 ticks), JAM (10 N), RIM (8 N × 2 within 2 mm of the rim), OFF_AXIS (peg face
    > 5 mm under the top, > 2.5 cm off the axis), BLOCKED: all retry the same pair from the **last
    good state** (≥ 3 mm above the rim, measured and commanded), at most 3 retries.
  - TIMEOUT (20 s): truncated, not terminal, and gets a new pair.
  - SUCCESS = tip ≥ push − 1 mm (29 mm now), ≤ 2 mm off the axis.
- **`--failure-mode intervene`** is a SERL intervention: the policy's bad frame gets the fail
  penalty, then the machine pulls out (mode 2), re-seats the hole with the edge press (mode 7), and
  the expert inserts (mode 6). These are INTERVENTION frames (`is_intervention=True`), +10 s on the
  timeout.
- **Reset (~18 s, not recorded):**
  1. retract (30 mm/s in the bore, fast after);
  2. hole to a random pose;
  3. wait until the hole is still;
  4. **8 N edge press**, force-controlled, 12 mm beside the hole;
  5. peg to a random start.

  A retry reset keeps the hole, re-seats it, and puts the peg at the good state. **No gear guard in
  resets** (it runs in episodes only); the 6 mm hole-push watch stays.
- **Policy limits:** a box around the hole, and the peg is kept 5 mm above the rim more than
  2.5 cm off the axis. Reset moves are exempt.

### Physical setup — changed 2026-10-05
The hole block **slipped in the left gripper**: 5.8 mm lower, ~1.8 mm sideways. It was handled by:
- a new eye calibration (`recordings/peg_hole_align/20261005_005434` + manual alignments, pool 30) →
  `config/peg_hole_offset_model.json` (const −0.87 / −1.61 mm);
- `config/peg_hole_tcp.yaml`, lift −0.30 overrides: **`top_m: 0.0476`**, **`push_m: 0.030`** (35 mm
  reached the bottom part).

The old model and pool are kept as `*_20261004.json`. **If the block is re-seated:** remove the two
yaml lines and recalibrate by eye (`peg_hole_align_gui.py --hover 0.0005`).

### Demos
- **`recordings/peg_hole_demos/expert_v2_20261005`: 100/100 first try**, peak force median 1.0 N,
  max 4.5 N, 10 242 frames, both wrist images on every frame. These are the ones to train with.
- Archived, from the old block position or aborted: `expert` (10-04), `expert_v0_before_seat_20261004`,
  `expert_v2_cornerpush_aborted`, `expert_v2_video_compromised`, `expert_v2_restart2_aborted`,
  `expert_v2_push35_aborted`.

### Tools
| tool | what |
|---|---|
| `peg_hole_demo_recorder.py` | expert demos: Sobol starts, auto retry, manual calibration on failure, resumable, `--no-manual` test mode |
| `peg_hole_expert_poke.py` | random reset, hold, run the expert on a trigger (diagnosis) |
| `peg_hole_align_gui.py --hover` | eye calibration of the peg offset |
| `peg_hole_waypoint_gui.py` + `--seat-path` | teach hole-relative waypoints and replay them in resets (tried for drags, not used now) |
| `peg_hole_roi_gui.py` | wrist-camera ROIs → `config/peg_hole_roi.json` |
| `tools/view_demo.py` | demo viewer (speed, n/p keep the time position) |
| `tools/export_demos.py` | demos → hil-serl transition pkl |
| `tests/test_server_local.py` | the real server against a fake robot + ZMQ client (protocol, retries, interventions, lockstep) |
| `tests/fake_robot_server.py` | the real server code on the network with a fake robot and synthetic images (client dry runs) |
| `tests/serl_live_client.py` | minimal live client (`--reset-mode`) |

### Client kit
`~/peg_hole_serl_kit_20261005.zip`: wire.py, README/CHANGES/FRAME, the examples, and the 100 new demos.

## Live results (2026-10-05)
- Expert with the edge-press reset: 5/5 first try at 0.3–1.5 N.
- Retry: new pair → JAM → retries 1–3 from the good state (3.0–3.2 mm above the rim) → cap.
- Intervention: bind 9 N → pull out → seat 8.8 N → expert → SUCCESS in the same episode.
- Client dry run against the fake-robot server: after the client's "processes" fix, **97 %** of
  ticks got a fresh delta (median age 63 ms = one tick).
- **First live client episode:** TIMEOUT (the peg ended 5 mm above the rim). Only 90 deltas came in
  300 ticks (30 %), then **the client stopped**: no further RESET.

## What remains
1. **Client (blocking):**
   - why the actor stopped after episode 1;
   - the slow lockstep with real images (30 % fresh);
   - the missing `if __name__ == "__main__":` guard (a second actor would drive the same arm);
   - confirm the learner is optimising (unbuffered logs).
2. **Repo:** the server imports `peg_hole_teach.py`, `peg_hole_random.py` and `peg_hole_stroke.py`,
   and reads `config/peg_hole_tcp.yaml`. **None of them are in git**, and `peg_hole_teach.py` has
   other people's work mixed in. Decide what gets committed before anyone else checks out the branch.
3. **Block mounting:** re-seat it properly or accept the current calibration. Expert insertions are
   clean now, but any further slip invalidates the calibration and the demos. Recalibrate by eye
   after any hardware change.
4. **Known quirks:**
   - the left (hole) arm can drift 10–16 mm early in an episode (IK coupling when the right arm
     moves fast);
   - the force estimate is ±5 N in free air while moving;
   - the edge press moves the hole 0.6–1.7 mm while seating.
5. **Older items:**
   - the gateway's SetMode clashes with msg type 6;
   - the SpaceMouse limit profile does not clamp scripted goals;
   - `limit_profiles.txt` has uncommitted changes from safe setup;
   - `REAL_ROBOT_README.md` "Remaining plan" is partly superseded by this file.
6. **Then: training.** Short supervised runs first: terminate + RESET AUTO, someone at the robot.
   Add interventions (human SpaceMouse on the client, or `--failure-mode intervene`) once the loop
   is stable.

## Running it
```bash
# robot workstation (ROS sourced, ROS_DOMAIN_ID=30); keep peg_hole_safe_setup.py running (lift lock, limit frames)
cd ~/robotis_ws/src/ai_worker/ffw_collision_checker/scripts
.venv/bin/python peg_hole_serl_server.py --allow-motion --images --port-base 7601
# client: SUB tcp://192.168.0.241:7601, PUB tcp://192.168.0.241:7602 (client machine 192.168.0.249)
```
