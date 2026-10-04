# Frame (msg type 14) — real-robot peg-hole server

One `Frame` per 15 Hz tick on PUB port base+0 (`peg_hole_serl_server.py`). Little-endian, 2646 bytes:

| bytes | content |
|---|---|
| 0:12 | header: int32 type = 14, float64 timestamp (server `time.time()`) |
| 12:996 | **obs**: 123 float64 — what a deployed actor can measure |
| 996:2580 | **priv**: 198 float64 — everything the server derives (critic inputs, logging) |
| 2580:2646 | **tag**: uint32 episode_id, uint32 step, uint8 frame_state, uint8 reason, float64 reward, 6 float64 action |

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
capped at 10 s (= none). Type-12 frames (the 2026-10-04 demos, NaN for "n/a") still
decode: `decode_frame` upgrades them to this layout. Velocities are finite differences between consecutive frames, angular
velocity as a vector (not rpy rates). Limit margins: the IK EE pose in the arm's
`peg_hole_frame_<l|r>` (static TF from `peg_hole_safe_setup.py`) against the
`peg_hole_safe_real_<l|r>` box in `ffw_collision_checker/config/limit_profiles.txt`;
`NO_LIMIT_DIFF` = 10.0 on an arm whose frame is not on TF (see
`priv.limit_frame_ok`).

Tag `frame_state`: 0 IDLE, 1 POLICY, 2 INTERVENTION, 3 TERMINATED, 4 RESET, 5 FAULT. Store POLICY and
INTERVENTION frames, TERMINATED ends the episode, skip RESET / IDLE / FAULT. `reason` (on
TERMINATED): 1 SUCCESS, 2 JAM, 3 RIM, 4 BLOCKED, 5 TIMEOUT, 6 SAFETY, 7 INTERVENTIONS, 8 ABORT, 9 OFF_AXIS, 10 BIND. `action` = the
right-arm delta actually applied this tick (the policy's after clipping, or the machine's on
INTERVENTION frames); `priv.policy_delta` is what the policy sent (`policy_delta_valid`).

### obs (123 float64)

| index | name | size | meaning |
|---|---|---|---|
| 0:75 | `joints` | 75 | per joint [pos, vel, effort], interleaved: joint j at 3j..3j+2; effort in A on the arm joints, driver units elsewhere |
| 75:81 | `ee_right` | 6 | right IK EE pose, achieved (/ik_solver/achieved_ee_pose_r), base_link |
| 81:87 | `ee_right_vel` | 6 | right EE velocity (vx, vy, vz, wx, wy, wz), base_link, finite difference |
| 87:93 | `ee_left` | 6 | left IK EE pose, achieved, base_link |
| 93:99 | `ee_left_vel` | 6 | left EE velocity |
| 99:111 | `limit_right` | 12 | right EE margins to the peg_hole_safe_real_r box in peg_hole_frame_r: x_lo, x_hi, y_lo, y_hi, z_lo, z_hi, roll_lo, ..., yaw_hi; lo = v - min, hi = max - v (< 0 = past the bound); NO_LIMIT_DIFF = no box / frame |
| 111:123 | `limit_left` | 12 | left EE margins, peg_hole_safe_real_l in peg_hole_frame_l |

### priv (198 float64)

| index | name | size | meaning |
|---|---|---|---|
| 0 | `reward` | 1 | this tick's reward (= tag.reward) |
| 1 | `return` | 1 | episode return so far, this tick included |
| 2 | `r_success` | 1 | reward term: success bonus (terminal) |
| 3 | `r_shape` | 1 | reward term: potential-based progress |
| 4 | `r_force` | 1 | reward term: push-back penalty |
| 5 | `r_fail` | 1 | reward term: failure penalty (terminal) |
| 6 | `push_back` | 1 | F_pb: push-back on the peg along its axis, N (> 0 = pushed back) = ref - axial |
| 7 | `push_back_peak` | 1 | highest F_pb since the peg could first touch the hole, N |
| 8 | `force_axial` | 1 | raw axial force estimate before the reference, N |
| 9 | `force_ref` | 1 | free-motion reference (median while the peg cannot touch), N; 0 until known |
| 10 | `force_ref_valid` | 1 | 1 = force_ref (and so push_back) is established |
| 11 | `depth` | 1 | peg tip below the rim, m (< 0 = above), measured |
| 12 | `depth_cmd` | 1 | the same for the commanded pose, m |
| 13 | `lateral` | 1 | peg tip distance from the calibrated hole axis, m |
| 14:16 | `lateral_yz` | 2 | the same as (y, z) components in the hole tool frame, m |
| 16 | `tilt` | 1 | angle between peg and hole axes, deg |
| 17:23 | `hole_pose` | 6 | hole tool point pose (x = insertion axis, up), base_link |
| 23:29 | `peg_in_hole` | 6 | peg tool pose in the hole tool frame (measured) |
| 29:35 | `ee_right_cmd` | 6 | right commanded IK EE pose after the server's clamps, base_link |
| 35:37 | `hole_offset` | 2 | calibrated peg-tool (y, z) offset in use, m |
| 37:43 | `policy_delta` | 6 | the policy's raw delta used for this tick; 0 when none / stale (policy_delta_valid) |
| 43 | `policy_delta_valid` | 1 | 1 = a fresh policy delta drove this tick |
| 44 | `delta_age` | 1 | s since the newest policy delta arrived, capped at 10 (10 = none yet) |
| 45 | `clamped` | 1 | 1 = the server's box / above-rim clamp changed the command this tick |
| 46 | `mode` | 1 | who drives the NEXT tick: 0 idle, 1 policy, 2 pull_out (machine), 4 reset, 5 fault, 6 expert (machine intervention to the goal); 3 trace_back retired |
| 47 | `interventions` | 1 | machine interventions so far this episode (--failure-mode intervene only) |
| 48 | `reset_kind` | 1 | how this episode started: 0 a new random pair, 1 a retry from the parent's good state |
| 49 | `retry_count` | 1 | retries of this hole / peg pair so far (0 = a new pair) |
| 50 | `parent_episode` | 1 | the episode this one retries (0 = none) |
| 51 | `t_episode` | 1 | s since the episode started |
| 52 | `can_touch` | 1 | 1 = the peg can touch the hole (tip within 1 mm of the rim or lower) |
| 53 | `in_zone` | 1 | 1 = within the 2 cm interaction zone of the calibrated axis |
| 54 | `success` | 1 | 1 = success condition met this tick |
| 55 | `failed` | 1 | 1 = reward-rule failure (jam / rim) this tick |
| 56 | `rim_strike` | 1 | 1 = the failure was a rim strike |
| 57 | `blocked` | 1 | 1 = blocked progress (command descends, peg does not) |
| 58 | `safety` | 1 | 1 = live safety stop this tick (gear guard, hard edge, left j7, hole pushed) |
| 59 | `hole_shift` | 1 | hole moved sideways since the peg was last clear, m |
| 60 | `j7_left_change` | 1 | |left j7 current change| since the peg was last clear, mA (gear guard input) |
| 61:63 | `limit_frame_ok` | 2 | 1 = peg_hole_frame_r / _l found on TF (obs limit_* valid) |
| 63:65 | `image_age` | 2 | s between receiving the right / left wrist image and this frame, capped at 10 (10 = none) |
| 65:67 | `image_ok` | 2 | 1 = a right / left wrist image is paired with this tick |
| 67:198 | `gateway_obs` | 131 | the gateway's 131-double Obs (decode_gateway_obs() names it): joint blocks, ee + grippers, marker block |

