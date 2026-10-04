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

- [ ] 0 contract  - [ ] 1 logic  - [ ] 2 idle server  - [ ] 3 policy + reset
- [ ] 4 intervention  - [ ] 5 images
