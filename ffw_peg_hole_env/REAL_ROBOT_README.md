# Peg-in-hole HIL-SERL on the real robot

Status 2026-10-04. The real-robot server speaks the HIL-SERL ZMQ wire, runs episodes
automatically (reset → policy → machine intervention → termination → reset) and has been
live-tested for safety; a full successful learning loop has not run yet. The MuJoCo sim
server (`README.md`) is parked: it works, but all current work targets the robot.
Day log with every number behind the decisions: `SERL_REAL_PLAN.md`.

## Pieces

| file | what |
|---|---|
| `ffw_collision_checker/scripts/peg_hole_serl_server.py` | the real-robot server (ROS node on the workstation, drives the IK through the teach tool's streamer) |
| `ffw_peg_hole_env/episode.py` | per-tick episode logic: frame state, termination, machine intervention (no ROS) |
| `ffw_peg_hole_env/reward.py` | reward from the right arm's currents and the peg/hole poses |
| `ffw_peg_hole_env/effort.py` | raw `/joint_states` effort → A → joint torque (load-cell driving K) |
| `ffw_peg_hole_env/wire.py` | wire messages (standalone: struct + numpy) incl. `Frame` (type 12) and its field tables |
| `ffw_peg_hole_env/observation.py` | Frame obs block: interleaved joints, EE velocity, limit-box margins (no ROS) |
| `FRAME.md` | Frame layout, field by field (generated: `tools/write_frame_md.py`) |
| `ffw_zmqinterface/gateway_node.py` | robot gateway: arm effort now converted to A in `Obs` |
| `ffw_collision_checker/scripts/peg_hole_safe_setup.py` | lift −0.30 m, lift hard-lock, per-arm EE limit profile from the real geometry |
| `ffw_collision_checker/scripts/peg_hole_align_gui.py` | eye-alignment GUI over a 25-pose grid (random height + tilt) → the peg offset |
| `ffw_collision_checker/config/peg_hole_offset_model.json` | calibrated peg offset (constant + position/height/tilt fit) |
| `ffw_collision_checker/scripts/peg_hole_demo_recorder.py` | 100 expert demos through the server (Sobol starts, fitted offset, manual recalibration on failure) |
| `ffw_collision_checker/config/peg_hole_offset_pool.json` | eye-alignment pool the offset model is fit on (created on the first recorder run) |
| `ffw_collision_checker/scripts/peg_hole_roi_gui.py` | wrist-camera ROI picker (both D405 RGB off UDP) → `config/peg_hole_roi.json` |
| `ffw_peg_hole_env/images.py` | ROI crop + downsample → 128×128 RGB policy image (no ROS) |
| `ffw_collision_checker/scripts/peg_hole_teach.py` | teach tool (`--plane-scan`, `--plane-verify`, `--model-verify` added; untracked in git) |
| `tests/test_episode.py`, `tests/test_wire_and_task.py` | unit checks |
| `tests/serl_live_client.py` | live test client (`--policy hold` / `down`) |

## Run

```bash
cd ~/robotis_ws/src/ai_worker/ffw_collision_checker/scripts     # ROS sourced, ROS_DOMAIN_ID=30
.venv/bin/python peg_hole_safe_setup.py                 # once: lift -0.30, lock, limit profile (keep it running)
.venv/bin/python peg_hole_serl_server.py --port-base 7601                    # wire check: IDLE frames, no motion
.venv/bin/python peg_hole_serl_server.py --allow-motion --port-base 7601     # real episodes after EnvCmd RESET
#   --autostart (no RESET needed)   --start-xy 0.002 (tests: peg starts near the calibrated axis)
#   --images (decode the wrist D405s: priv image_t; the ffw_stream receiver must not run)
.venv/bin/python peg_hole_roi_gui.py                    # once: wrist-camera ROIs -> config/peg_hole_roi.json
.venv/bin/python peg_hole_demo_recorder.py --allow-motion   # expert demos (resumable; Enter/s/q in the teach window)

cd ../../ffw_peg_hole_env
../ffw_collision_checker/scripts/.venv/bin/python tests/serl_live_client.py --policy down --episodes 2
../ffw_collision_checker/scripts/.venv/bin/python tests/test_episode.py
```

Stop the server with `q` / space in the teach window or by PID (`kill <pid>`); never
`pkill -f` with a pattern that also matches the calling shell.

## Wire (ports = base + n)

| port | socket on server | messages |
|---|---|---|
| +0 | PUB | `Frame` (type 12), one per 15 Hz tick |
| +1 | SUB | `ControlCmdDelta` (6), `EnvCmd` (8) |

Every frame starts with the gateway header (int32 type, float64 timestamp).

**Frame (12), 2590 bytes — full layout in [`FRAME.md`](FRAME.md):** obs (123 float64: 25 joints ×
[pos, vel, effort] interleaved, both IK EE poses + velocities in base_link, both arms' margins
to their `peg_hole_safe_real` box), priv (191 float64: reward + terms + return, push-back force
estimate, task geometry, env flags, the gateway's full 131-double Obs, …), then the tag:
`episode_id, step` (uint32), `frame_state, reason` (uint8), `reward` (float64),
`action` (6 float64: the right-arm delta actually applied this tick — the policy's, or the
machine's on INTERVENTION frames). `wire.decode_frame()` returns (obs, priv, tag, ts) as
named arrays. The server packs everything; the client picks actor / critic inputs. One
message per tick, so obs, priv and label can never be paired wrongly. Type 11 (the
gateway Obs + tag) is retired. The limit frames are looked up on TF once: restart the
server after re-running `peg_hole_safe_setup.py`.

| frame_state | meaning | HIL-SERL |
|---|---|---|
| POLICY (1) | the policy's delta was applied | store |
| INTERVENTION (2) | the machine drove the arm (pull out, trace back) | store as intervention |
| TERMINATED (3) | last frame; `reason` says why | store, done |
| RESET (4) | between episodes | skip |
| IDLE (0) / FAULT (5) | waiting / reset failed — check the robot | skip |

Reasons: SUCCESS 1, JAM 2, RIM 3, BLOCKED 4, TIMEOUT 5, SAFETY 6, INTERVENTIONS 7, ABORT 8
(OFF_AXIS 9 reserved, no longer sent).

**ControlCmdDelta (6):** 14 float64, right (dx, dy, dz, droll, dpitch, dyaw, grip) then left.
Applied to the commanded right IK EE pose (priv `ee_right_cmd`; obs `ee_right` is the achieved one): translation in
base_link, rotation R_new = R_delta·R (intrinsic X-Y-Z, the `Obs` rpy convention). Clipped
per axis to 6.67 mm / 2° per tick (= action ±1). No delta for 0.2 s = hold. Left and grip
are ignored.

**EnvCmd (8):** RESET starts the automatic episode loop (`episode_id` sets the next id),
PAUSE stops after the current episode (also during a reset), RESUME, ABORT ends the
episode now (TERMINATED ABORT, then IDLE).

**Obs effort is amps** on every arm joint, real and sim: the gateway converts the driver's
raw units (YM080 ÷13 and YM070 ÷25 in 0.01 A counts, PH42 J7 in mA). Other joints stay in
driver units. Torque = load-cell **driving K** (K·η), one constant per joint; never switch K
by motion state (gravity current turns into fake ~10 N forces).

## An episode, all automatic

1. **RESET:** lift the peg straight up the hole axis to clear height (hole watched),
   then the teach tool's reset (random reach-checked hole pose, peg start). A lag trip, a
   pushed hole or a gear-guard trip → FAULT, arms held, no retries.
2. **POLICY:** each delta is applied, clamped to a box around the hole, and — farther than
   2 cm from the calibrated axis — kept 5 mm above the rim.
3. **INTERVENTION** (the machine, not a human): pull the peg straight up the axis until its
   tip is 5 mm clear, trace back to the last policy pose near the hole top (or the peg
   aligned on the calibrated axis), then the policy continues, same episode. Max 3 per
   episode. Triggers, only while the peg can touch the hole (tip within 1 mm of the rim
   or lower):
   - tip below the rim more than **2 cm** off the calibrated axis (heading down beside the block);
   - push-back > **7 N for 2 ticks** (was 6 N on one tick: it stopped normal chamfer entries);
   - blocked progress (command descends, peg does not);
   - left j7 current change > **300 mA** since the peg was last clear.
4. **TERMINATED:**
   - SUCCESS — tip 34 mm in, within 2 mm of the calibrated axis;
   - JAM / RIM — push-back > 10 N, or > 8 N for 2 ticks within 6 mm of the rim;
   - SAFETY — hole pushed > 6 mm sideways, gear guard, hard edge (25 N), left j7 > 450 mA;
   - INTERVENTIONS / BLOCKED — a 4th trigger;
   - TIMEOUT — 20 s.

**Interaction zone:** within 2 cm of the axis the peg may touch the block top, chamfer and
rim freely (inside the block footprint it cannot get beside the block); the force rules and
the gear guard limit that contact.

**Gear guard,** every control tick while anything is commanded (episode, intervention,
retract, reset): left j7 change from its unloaded reference > 450 mA at once, or > 300 mA
for 0.35 s. During a retract only a rising load trips. Recorded clean pushes peak ≤ 400 mA
and never stay > 300 mA for more than 0.30 s; on the incident replay it trips before the
sideways drag starts.

## Reward (per tick)

| term | value |
|---|---|
| success | 1 + 0.5·(1 − F_peak/10 N): 0 N → 1.5, 4 N → 1.3, 10 N → 1.0 (F_peak = highest push-back since contact was possible) |
| shape | potential-based progress (lateral, tilt, depth-to-go), ≤ +0.3 per episode |
| force | −0.01·clip(F_pb/10 N, 0, 1), no dead zone |
| fail | −0.5 − 0.5·(share of 34 mm not reached): deep jam ≈ −0.5, stuck halfway ≈ −0.75, rim ≈ −1 |

F_pb = push-back on the peg along its axis from the right arm's currents (driving K, weighted
Jᵀ solve at `right_peg_site`), minus a reference held while the peg cannot touch the hole.
It reads like the teach GUI's force numbers. On recorded pushes the ordering is clean
(+1.0…1.4) > deep jam (−0.5) > stuck shallow (−0.7) > rim (−0.8).

## Peg offset calibration

The peg must be offset from the modelled hole axis. Force-based searches (offset search,
plane scan, plane verify) were bounded at ±1.5 mm and gave misleading answers — the true
offset is outside that range.

| step | result |
|---|---|
| eye alignment, 25 poses (`peg_hole_align_gui.py`, `recordings/peg_hole_align/20261004_152716`) | peg-tool y **−1.84** ± 0.32, z **−3.08** ± 0.68 mm; residual tilt ≤ 0.02° |
| linear model (position, height, tilt) | z predicted to 0.30 mm leave-one-out (constant 0.71) — but not better on the robot |
| model verify, 10 new random poses (±4 cm, height 0…−2 cm, tilt ±3°) | model 20/20 full depth, median 7.4 N; constant 20/20, 7.5 N (before: 24–68 %, 10–25 N) |

Use the constant (−1.84, −3.08) mm (`config/peg_hole_offset_model.json`, `"const"`). The
server measures every on-axis rule from the axis corrected by it (uncorrected, good pushes
read up to 4.4 mm off). After any hardware change, recalibrate by eye with the GUI, not by
a force search. The hole axis is ~3° off the tool model (z offset grows ~1 mm per 20 mm of
height) — not worth fixing at this tolerance.

## Safety lessons (2026-10-04 incident)

First live run: the peg started 65 mm off axis, the test client pushed straight down, the
peg went below the rim beside the block (scraping it), and the teach tool's reset ("realign
at the current height", which assumes a peg below the rim is in the hole) dragged it
sideways into the block — hole arm pushed 88 mm, left j7 pinned at 1.5 A for ~40 s. The
user attributes the changed peg offset to that. What prevents it now: the 2 cm zone with
the above-rim clamp, the halt + intervention below the rim outside it, the straight-up
retract before every reset, the hole-push watch, the gear guard, and FAULT without retries.

Also: `peg_hole_safe_setup.py`'s limit profile clamps **SpaceMouse goals only**; scripted
goals on `/quest/<arm>/ee_target_pose` (teach tool, this server) are not clamped by it.

## Remaining plan

**Next (agreed 2026-10-04), in order:**

A. **Wrist-camera ROI** — done: `peg_hole_roi_gui.py` decodes both wrist D405 RGB feeds off
   UDP (left 9001, right 9003, 270×480 after the CCW rotation; the ffw_stream ROS receiver must
   not run), you place one square ROI per camera (default 256 px), saved to
   `config/peg_hole_roi.json`; `ffw_peg_hole_env/images.crop()` → 128×128 RGB (2×2 area).
B. **Frame reorganised for asymmetric actor-critic** — done (type 12, `FRAME.md`; unit-tested,
   not yet run live). The server packs everything, documented
   field by field; the client picks actor vs critic inputs.
   - obs: 25 joints × `[pos, vel, effort]` interleaved per joint (75); right, left EE pose +
     velocity in base_link (24); per arm 6 axes × (lo, hi) distance to the
     `peg_hole_safe_real_<arm>` box in `peg_hole_frame_<arm>` (24) — computed by the server,
     because the gateway's `diff0/diff1` only accept `marker_frame` and read 10.0 here.
   - priv: F_pb estimate + reference, F_peak, reward terms + sum, env flags, hole pose, raw
     131-double gateway Obs, image receive times, anything else the server derives.
C. **Expert demo recorder** — written (`peg_hole_demo_recorder.py`; pool/fit/Sobol checked offline,
   not yet run on the robot): 100 start configurations on an evenly spaced grid over the reset
   space; fitted-offset scripted insertion through the server, saved locally (`Frame`s + both
   128×128 images per tick). Any non-SUCCESS → align-GUI mode at that pose (you adjust, Enter
   registers it into the offset pool, refit) → retake. Only SUCCESS counts; resumable.
   Streaming images to the client during training comes after.

Older items:

1. **Success test of the full loop:** a test client that descends along the hole axis
   (straight base −z drifts ~1.7 mm across the 5°-tilted axis over 20 mm), with near-axis
   starts (`--start-xy`), to show reset → policy → insert → SUCCESS → reset, and the
   intervention cycle when it binds.
2. **Images:** the wrist cameras on each `Frame` tick (sim used 128×128 RGB; the robot has
   IR / depth — decide the format and pairing).
3. **HIL-SERL client update** (their side): decode `Frame` (type 12, 2590 bytes, `FRAME.md`) instead of
   separate `Obs` + `EnvStatus`; store by `frame_state`; full 6-DOF action passthrough (the
   stock delta step zeroes rotation); send `EnvCmd RESET` once to start.
4. **Gateway protocol:** move `SetMode` off msg type 6 (clashes with `ControlCmdDelta`).
5. **Reset start offset:** use the calibrated offset in the teach tool's auto pushes / reset
   peg starts, and keep the reset box out of regions that never insert.
6. **Policy-time limits:** scripted goals bypass the joy_hand profile — consider applying the
   same clamp in the shared goal publisher (`peg_hole_random.Io.publish_goal`).
7. **Housekeeping:** `peg_hole_teach.py` is untracked with others' work mixed in (plane scan,
   verify modes, `hold` fix live there); the `limit_profiles.txt` change is uncommitted.

## Commits (branch `radio_teleop_moveit`)

`68d287e` env baseline · `63bceea` Frame message · `73e0e7a` episode logic ·
`c0a38d3` idle server · `f4ca248` real-robot contact rules · `72e83ee` server slices 3–4 ·
`09eb310` safe setup · `f4c2bfc` gear guard + calibrated axis · `a7246a0` align GUI + offset
model · `fde04fd2` OFF_AXIS stop (superseded) · `d3540f8c` 2 cm interaction zone + live fixes.
