# HIL-SERL on the real robot: spec, slices, status

Handoff written 2026-10-04 before a context clear. Sim side is parked (it works:
`README.md`, `HILSERL_CLIENT_REQUEST.md`); everything below is the REAL robot.

## Context (what was decided / learned on 2026-10-04)

**Effort / force.** `Obs.joint_effort` is amps on every arm joint: the gateway
(`ffw_zmqinterface/gateway_node.py` `_effort_to_amps`) converts the driver's raw
units (YM080 /13, YM070 /25 in 0.01 A counts, PH42 J7 in mA). Torque = load-cell
**driving K = K * eta**, one constant per joint (`effort.py`). Do NOT switch K by
motion state: the arm holds tens of N m of gravity, and switching turns it into fake
~10 N force jumps (free-air sd 1.5 N switched vs 0.46 N constant). Push-back F_pb
(right arm, J^T at `right_peg_site`, `reward.PushBackForce`) reads like the teach
GUI's numbers (corr 0.93 over 106 pushes).

**Reward** (`reward.py`, final): success = tip 34 mm in -> 1 + 0.5*(1 - F_peak/10 N)
(0 N 1.5, 4 N 1.3, 10 N 1.0); shaping <= 0.3 per episode; force -0.01*F/10 per step;
fail (only while the peg can touch the hole) at F_pb > 10 N or > 8 N for 2 ticks within
6 mm of the rim = -0.5 - 0.5*(share of 34 mm not reached). Ordering on recorded
pushes: clean +1.0..1.4 > deep jam -0.5 > stuck shallow -0.7 > rim -0.8.
Replay caveat: recordings that start the push at the rim have no free-air zero;
the pre_push baseline drifts up to ~10 N with pose, so they can't be scored fairly.

**Calibration finding.** The user's gentle manual pushes (3-6 N, session
`20261004_090934`) used a hand offset of peg y -1.17, z -1.51 mm (1.9 mm across
the axis); the 50 straight-on 35 mm pushes (`20261004_023307`) jammed > 10 N deep.
Plane scan running (`peg_hole_teach.py --plane-scan`, session `20261004_134329`):
4x4 hole poses across the axis, per pose a compass search of the peg offset
(+-1.5 mm, 0.25 step) for a full push < 10 N. First 7 poses: 3 found, all at
9.4-10 N; best y keeps hitting the -1.5 mm bound; confirms fail often (10 N is
inside the push-to-push noise). Next time: stepwise (no --continuous) and +-2.5 mm.
Results -> `<session>/plane_scan.json`.

## Spec: per-frame state (agreed with the user)

One message per 15 Hz tick, the Obs payload plus a tag, so the label can never be
paired with the wrong frame:

| frame_state | meaning | HIL-SERL |
|---|---|---|
| POLICY | the policy's delta was applied | store |
| INTERVENTION | the MACHINE took over (not a human): pull the peg out along the hole axis, then trace back to the last pose near the hole top; action = the machine's delta | store as intervention |
| TERMINATED + reason | last frame (success / jam / rim / blocked / timeout / safety) | store, done |
| RESET | between episodes (peg out, new random pose, reach check, move) | skip |
| IDLE / FAULT | waiting / reset failed repeatedly | skip |

All automatic on the robot:
- **Intervention trigger:** F_pb > ~6 N (below the 10 N fail), blocked progress,
  or too far off axis near the rim. Ends at the restored near-top pose (a ring
  buffer of peg poses with the tip a few mm above the rim, on the hole side); the
  policy continues in the same episode. Max 3 per episode, then TERMINATED (fail).
- **Termination:** reward.py rules + the teach tool's live stops (j7 stall, hard
  edge) + 20 s timeout.
- **Reset:** `peg_hole_teach.Teach.hw_reset` (lift out, sample, reach check, move).

## Slices

0. **Contract** - `wire.py`: FRAME message (header + 131-double Obs payload + tag:
   episode_id, step, state, reason, reward, applied 6-DOF action), encode/decode,
   unit test.
1. **Episode logic, no ROS** - a per-tick state machine (POLICY / INTERVENTION
   stages / TERMINATED / RESET) fed with depth, lateral, F_pb, blocked, safety,
   time; returns the frame tag and what the robot should do. Tested offline on
   recorded pushes and synthetic traces.
2. **Real server, idle** - ROS node in `ffw_collision_checker/scripts` (reuses
   `peg_hole_teach` Teach/TeachIo/EdgeForce/BlockDetector): publishes FRAME at 15 Hz
   (IDLE), receives ControlCmdDelta / EnvCmd. No motion.
3. **Policy deltas + termination** on the robot, reset between episodes.
4. **Machine intervention** motion (pull out, trace back), frames tagged.
5. Images (wrist cameras) on the FRAME tick.

Out of scope now: sim parity, human SpaceMouse intervention.

## Status

- [x] 0 contract  - [x] 1 logic  - [x] 2 idle server  - [x] 3 policy + reset (code)
- [x] 4 intervention (code)  - [ ] 5 images

Slice 1 notes: `episode.py` EpisodeMachine + `tests/test_episode.py`. Replayed on
recordings: gentle manual pushes (`20261004_090934`) 4/4 SUCCESS, no intervention;
the 35 mm jam session (`20261004_023307`) triggers on every push -- ~30 deep at
22-24 mm (push-back rising through 6 N, as intended) and ~20 at the entry (3-6 mm,
5-7 N normal entry bumps). `f_intervene` (6 N) is the knob for the first live run.

## 2026-10-04 live incident and the rules it added

First live run (`recordings/peg_hole_serl/20261004_142240`): the reset put the peg 65 mm
off the axis, the test client pushed straight down, the peg went 39 mm below the rim
BESIDE the block (scraping it, left j7 to 453 mA -> j7 stop), and the teach tool's reset
("realign at the current height", which assumes a peg below the rim is IN the hole)
then dragged it sideways into the block: the hole arm was pushed 88 mm, j7 pinned at
1.5 A. No intervention fired: the sim's 41.5 mm block-footprint test called it "clear",
and the axial push-back does not see side contact.

Added (episode.py, peg_hole_serl_server.py), thresholds from the recordings (measured
from the last tick the peg was clear; clean pushes: hole sideways p95 2.2 / max 4.2 mm,
j7 p95 ~200 mA; stalls 300-460 mA):
- real robot "can touch" = tip within 1 mm of the rim or lower, at any offset;
- intervention also on: tip below the rim > 3 mm off axis; left j7 change > 300 mA;
- SAFETY (any mode): hole pushed sideways > 6 mm; plus the live j7 450 mA / hard edge;
- command clamp: farther than 3 mm off the axis the peg stays 2 mm above ON_TOP;
- reset: peg straight up the axis to CLEAR first, hole watched (> 6 mm -> stop); any
  lag trip or pushed hole -> FAULT at once, no retries.
Replays: the incident would have been caught at 1.11 s (at the rim, 61 mm off axis,
before any scraping) instead of the 3.08 s j7 stop; gentle pushes still 4/4 SUCCESS; no
new triggers elsewhere.

Safety setup (`ffw_collision_checker/scripts/peg_hole_safe_setup.py`): lift -0.30 +
hard lock + per-arm global limit profile from the real geometry ([peg_hole_safe_real_l/r],
frames peg_hole_frame_l/r). joy_hand clamps SpaceMouse goals only; scripted
/quest/<arm>/ee_target_pose goals are NOT clamped by it.

Plane scan result (`recordings/peg_hole_teach/20261004_134329/plane_scan.json`): 3/16
poses under 10 N (all at 9.4-10 N), 170 pushes; the -y half sits at 9.5-11.8 N, the +y
side at 16-25 N.

## True peg offset and the gear guard (2026-10-04, later)

Eye alignment over 25 hole poses (`peg_hole_align_gui.py`, `recordings/peg_hole_align/20261004_152716`):
the true peg offset is peg-tool y -1.84 +- 0.32, z -3.08 +- 0.68 mm -- z is twice the +-1.5 mm every
earlier search used (why the plane scan pinned at its bound and the first verify got 12/50). A linear
model in position / height / tilt predicts z to 0.30 mm (constant 0.71), but on the robot it does not
beat the constant: model-verify on 10 new random poses (`recordings/peg_hole_teach/20261004_153905`),
model 20/20 full depth, median 7.4 N vs constant 20/20, 7.5 N (paired +0.17 +- 1.16 N). The constant
(-1.84, -3.08) is in `config/peg_hole_offset_model.json` ("const"); the server measures every on-axis
rule (success, beside-the-hole, floor clamp) from the axis corrected by it, and the plane scan
warm-starts there.

The user attributes the offset change to the incident (j7 pinned at 1.5 A ~40 s while the reset dragged
the peg into the block). Gear guard in the server, every control tick while anything is commanded
(episode, intervention, retract, reset): left j7 change from its unloaded reference > 450 mA at once,
or > 300 mA for 0.35 s -> SAFETY / FAULT; during a retract only a rising load trips. Replays: the
incident trips at 14.01 s (before the drag started ~14.4 s); 0/205 clean pushes trip.

OFF_AXIS stop (user, 2026-10-04): the tip below the rim more than 3 mm from the calibrated axis
ends the episode at once (TERMINATED, reason TR_OFF_AXIS = 9, fail ~ -1), in any mode; approaching
the rim off axis still triggers the machine intervention. Checks: the 40 model-verify pushes stay
<= 1.32 mm (median 0.24) from the calibrated axis below the rim (from the uncorrected axis up to
4.38 mm -- would all have been stopped); the incident stops at 1.14 s, 0.8 mm below the rim.
