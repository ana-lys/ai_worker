# Frame (msg type 12) — real-robot peg-hole server

One `Frame` per 15 Hz tick on PUB port base+0 (`peg_hole_serl_server.py`). Little-endian, 2590 bytes:

| bytes | content |
|---|---|
| 0:12 | header: int32 type = 12, float64 timestamp (server `time.time()`) |
| 12:996 | **obs**: 123 float64 — what a deployed actor can measure |
| 996:2524 | **priv**: 191 float64 — everything the server derives (critic inputs, logging) |
| 2524:2590 | **tag**: uint32 episode_id, uint32 step, uint8 frame_state, uint8 reason, float64 reward, 6 float64 action |

`wire.decode_frame(data)` → `(obs, priv, tag, ts)`: dicts of named numpy arrays (size-1 fields
as floats), each with `"vector"` = the whole flat block; `obs` also has `joint_pos`,
`joint_vel`, `joint_effort` (25 each). `wire.py` needs only struct + numpy.

The server packs everything; the client chooses what the actor and the critic see
(asymmetric actor-critic). obs and priv travel in the same message, so they can never be
paired with the wrong tick.

Conventions: base_link; poses (x, y, z, roll, pitch, yaw) with R = Rx·Ry·Rz (the Obs /
`ControlCmdDelta` convention); m, rad, s, N, A unless stated. Joints in
`wire.JOINT_STATE_NAMES` order. **NaN** = not available in this frame state (episode values
outside an episode). Velocities are finite differences between consecutive frames, angular
velocity as a vector (not rpy rates). Limit margins: the IK EE pose in the arm's
`peg_hole_frame_<l|r>` (static TF from `peg_hole_safe_setup.py`) against the
`peg_hole_safe_real_<l|r>` box in `ffw_collision_checker/config/limit_profiles.txt`;
`NO_LIMIT_DIFF` = 10.0 on an arm whose frame is not on TF (see
`priv.limit_frame_ok`).

Tag `frame_state`: 0 IDLE, 1 POLICY, 2 INTERVENTION, 3 TERMINATED, 4 RESET, 5 FAULT. Store POLICY and
INTERVENTION frames, TERMINATED ends the episode, skip RESET / IDLE / FAULT. `reason` (on
TERMINATED): 1 SUCCESS, 2 JAM, 3 RIM, 4 BLOCKED, 5 TIMEOUT, 6 SAFETY, 7 INTERVENTIONS, 8 ABORT, 9 OFF_AXIS. `action` = the
right-arm delta actually applied this tick (the policy's after clipping, or the machine's on
INTERVENTION frames); `priv.policy_delta` is what the policy sent.

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

### priv (191 float64)

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
| 9 | `force_ref` | 1 | free-motion reference (median while the peg cannot touch), N |
| 10 | `depth` | 1 | peg tip below the rim, m (< 0 = above), measured |
| 11 | `depth_cmd` | 1 | the same for the commanded pose, m |
| 12 | `lateral` | 1 | peg tip distance from the calibrated hole axis, m |
| 13:15 | `lateral_yz` | 2 | the same as (y, z) components in the hole tool frame, m |
| 15 | `tilt` | 1 | angle between peg and hole axes, deg |
| 16:22 | `hole_pose` | 6 | hole tool point pose (x = insertion axis, up), base_link |
| 22:28 | `peg_in_hole` | 6 | peg tool pose in the hole tool frame (measured) |
| 28:34 | `ee_right_cmd` | 6 | right commanded IK EE pose after the server's clamps, base_link |
| 34:36 | `hole_offset` | 2 | calibrated peg-tool (y, z) offset in use, m |
| 36:42 | `policy_delta` | 6 | the policy's raw delta received for this tick (NaN = none / stale) |
| 42 | `delta_age` | 1 | s since the newest policy delta arrived (NaN = none yet) |
| 43 | `clamped` | 1 | 1 = the server's box / above-rim clamp changed the command this tick |
| 44 | `mode` | 1 | machine mode driving the NEXT tick: 0 idle, 1 policy, 2 pull_out, 3 trace_back, 4 reset, 5 fault |
| 45 | `interventions` | 1 | machine interventions so far this episode |
| 46 | `t_episode` | 1 | s since the episode started |
| 47 | `can_touch` | 1 | 1 = the peg can touch the hole (tip within 1 mm of the rim or lower) |
| 48 | `in_zone` | 1 | 1 = within the 2 cm interaction zone of the calibrated axis |
| 49 | `success` | 1 | 1 = success condition met this tick |
| 50 | `failed` | 1 | 1 = reward-rule failure (jam / rim) this tick |
| 51 | `rim_strike` | 1 | 1 = the failure was a rim strike |
| 52 | `blocked` | 1 | 1 = blocked progress (command descends, peg does not) |
| 53 | `safety` | 1 | 1 = live safety stop this tick (gear guard, hard edge, left j7, hole pushed) |
| 54 | `hole_shift` | 1 | hole moved sideways since the peg was last clear, m |
| 55 | `j7_left_change` | 1 | |left j7 current change| since the peg was last clear, mA (gear guard input) |
| 56:58 | `limit_frame_ok` | 2 | 1 = peg_hole_frame_r / _l found on TF (obs limit_* valid) |
| 58:60 | `image_t` | 2 | receive time (time.time) of the right / left wrist image paired with this tick; NaN = none |
| 60:191 | `gateway_obs` | 131 | the gateway's 131-double Obs (decode_gateway_obs() names it): joint blocks, ee + grippers, marker block |

