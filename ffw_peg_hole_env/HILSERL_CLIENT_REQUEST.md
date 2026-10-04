> **Superseded 2026-10-04 by `SERL_INTERFACE.md`** (real robot, Frame type 12). This file describes the
> sim server's rev-2 protocol and is kept for history.

# Request to the HIL-SERL side: peg-in-hole client for the FFW server

From: robot workstation (`ai_worker/ffw_peg_hole_env`), 2026-10-02 (rev 2)
To: HIL-SERL env / client developer

The server side is built and tested in simulation. This is what it expects
from the HIL-SERL client. Rev 2 records the decisions from your reply (section
5), moves EnvStatus/Images to their own ports, and spells out the full
131-double Obs. `wire.py` is now standalone (struct + numpy only) and includes
`decode_obs()`.

## 1. What the server does

One ZMQ server, the same event handling for the MuJoCo simulator and (next)
the real robot. Only the timing differs:

- **sim, lockstep:** every `ControlCmdDelta` advances exactly one 1/15 s tick
  and the observation comes straight back, so the sim runs as fast as you
  send (measured ~240 steps/s over ZMQ with both wrist images).
- **real (and sim `--realtime`):** a fixed 15 Hz tick; deltas received during
  a tick are composed; no delta = hold.

Task per episode: RESET samples the hole and the peg start in a hole-centre
frame (z = insertion axis, up): hole x/y ±5 cm, z 0 → −2 cm; peg x/y ±5 cm,
tip 2–4 cm above the rim, ±5° tilt. The policy commands the right arm (peg) in
full 6-DOF; the left arm (hole) holds.

## 2. Wire (all frames start with the gateway's 12-byte header: int32 type, float64 timestamp)

| port | socket on server | messages |
|---|---|---|
| base + 0 (6001) | PUB | `Obs`, byte-identical to the robot gateway's, 131 doubles (layout below) |
| base + 1 (6002) | SUB | `ControlCmdDelta`, `EnvCmd` |
| base + 4 (6005) | PUB | `EnvStatus` |
| base + 5 (6006) | PUB | `Images` (multipart) |

base + 2 (6003, `Priv`) and base + 3 (6004, `Record`) stay the robot gateway's
and are not used by this server. `wire.PORT_*` holds the offsets.

**Obs** (type 1): 12-byte header + 131 float64 = 1060 bytes. Your pinned
2026-09-05 layout is the first 89 doubles; the gateway has since appended a
42-double marker-frame block.

| index | field |
|---|---|
| [0:25] | joint_pos, in `JOINT_STATE_NAMES` order (arm_l_joint1..7, arm_r_joint1..7, gripper_l_joint1, gripper_r_joint1, head_joint1, head_joint2, left_wheel_drive, left_wheel_steer, lift_joint, rear_wheel_drive, rear_wheel_steer, right_wheel_drive, right_wheel_steer) |
| [25:50] | joint_vel |
| [50:75] | joint_effort (A) |
| [75:81] | ee0 = **right** EE (x, y, z, rx, ry, rz), base_link, R = Rx·Ry·Rz |
| [81] | grip0 = right gripper, 0 open .. 1 closed |
| [82:88] | ee1 = **left** EE |
| [88] | grip1 = left gripper |
| [89:95] | base_marker: base_link pose in the AprilTag marker frame |
| [95:101] | ee0_marker: right EE pose in the marker frame |
| [101:113] | diff0: right EE margins to the active limit box (x_lo, x_hi, y_lo, y_hi, z_lo, z_hi, roll_lo, …, yaw_hi); 10.0 = no box |
| [113:119] | ee1_marker |
| [119:131] | diff1 |

In sim the marker block stays at its defaults (zeros, and 10.0 for the diffs).
`wire.decode_obs(frame)` returns all of it as named numpy arrays; it is
checked field by field against the gateway's own encoder.

| type | message | payload |
|---|---|---|
| 6 | `ControlCmdDelta` | 14 float64: right (dx, dy, dz, droll, dpitch, dyaw, grip), left (same) |
| 8 | `EnvCmd` | uint8 cmd (1 RESET, 2 PAUSE, 3 RESUME, 4 ABORT, 5 PING), int64 seed, uint32 episode_id, 8 float64 reserved |
| 9 | `EnvStatus` | uint8: state, backend (0 sim, 1 real), reset_ok, success, terminated, truncated; uint32: episode_id, step, deltas_received; float64: reward, depth, lateral, contact_force |
| 10 | `Images` | multipart `[header + uint32 episode_id, uint32 step, uint16 h, uint16 w, right RGB bytes, left RGB bytes]`, 128×128×3 uint8 |

States: 0 IDLE, 1 RESETTING, 2 READY, 3 RUNNING, 4 PAUSED, 5 DONE, 6 FAULT.

Reference encoders/decoders: `wire.py` (standalone: struct + numpy; on the
robot workstation it additionally encodes Obs with the gateway's module).

## 3. Episode flow the client should implement

```
reset(seed):
    send EnvCmd RESET(seed, episode_id)
    wait EnvStatus with this episode_id and state READY   (FAULT -> retry with another seed)
    read Obs (step 0) and Images (episode_id, step 0)
    latch the commanded right-EE pose from Obs.ee[0]
    return observation

step(action):                       # action in [-1, 1]^6 (right arm)
    delta = action * (0.00667 m, 0.00667, 0.00667, 2 deg, 2 deg, 2 deg)
    send ControlCmdDelta(right = (*delta, grip), left = zeros)
    wait EnvStatus with this episode_id and step = previous + 1
    read Obs and Images for that (episode_id, step)
    reward     = EnvStatus.reward
    terminated = EnvStatus.terminated      # success, or contact failure
    truncated  = EnvStatus.truncated       # 300 steps = 20 s
    return observation, reward, terminated, truncated, info
```

## 4. Contract

- **Delta semantics.** Applied to the commanded **IK end-effector** pose (the
  pose `Obs.ee` reports; ee0 = right, ee1 = left). Translation added in
  base_link. Rotation R_new = R_delta · R, R_delta from intrinsic X-Y-Z angles,
  the same convention as the `Obs` rpy (R = Rx · Ry · Rz).
- **Per-axis cap (agreed): ±1 action = the cap on that axis.** Each
  translation component ±6.67 mm, each rotation component ±2° per tick. At 15 Hz:
  10 cm/s per axis, 30°/s per rotation axis. The server clips per axis; clip
  the same way before sending so the delta you send is the delta applied.
- **Latch at READY.** The server re-latches its command reference at every
  RESET.
- **grip** is carried but ignored: peg and hole stay gripped
  (`Obs.gripper` ≈ 0.87 / 0.87).
- **Effort** in `Obs.joint_effort` is in amps on every arm joint, sim and robot
  (rev 3: the robot gateway now converts the driver's raw units; the sim uses
  the load-cell driving K, K · gear efficiency). Other joints stay in driver units.
- **Reward** (rev 3) is dense: success (tip 34 mm in, the full push) pays
  1 + up to 0.5 for a gentle insertion (0 N peak push-back → 1.5, 10 N → 1.0);
  potential-based progress shaping (≤ 0.3 per episode); a small per-step
  push-back penalty; and −0.5 to −1 (shallower = worse) + terminate on a jam
  (push-back > 10 N) or a rim strike (> 8 N within 6 mm of the rim).
  `EnvStatus.contact_force` now carries that push-back estimate on both
  backends. Success also requires the peg within 2 mm of the hole axis.
- **Images**: pair them with `EnvStatus` by (episode_id, step). Both cameras
  are upright (image up = the insertion axis); the peg camera sits 10 cm up and
  5 cm forward of the CAD D405 position, pitched 30° down.
- **Timing**: in lockstep sim, send the next delta only after the status of
  the previous step arrives.

## 5. Decisions (agreed 2026-10-02)

1. `ControlCmdDelta` keeps **msg_type 6**. The gateway's `SetMode` (also 6
   today) moves to a new number; to be done in `ffw_zmqinterface/protocol.py`
   together with every `SetMode` sender.
2. `EnvCmd` / `EnvStatus` / `Images` (8 / 9 / 10) accepted as specified.
3. `EnvStatus` and `Images` get their own ports (base + 4 / base + 5); nothing
   shares a port with `Priv` or `Record`.

Also noted from your reply: the stock `MapDeltaActionToRobotActionStep`
zeroes the rotation delta; this task needs the full 6-DOF delta, so that step
must pass all six components through (scaled by the per-axis cap).

Still being built on the server side: SpaceMouse intervention in sim with the
human command reported back as `intervene_action`, the handover message and
push assist; the real-robot backend; the sim/real switch.

## 6. Try it against the sim

```bash
cd ~/robotis_ws/src/ai_worker/ffw_peg_hole_env
../ffw_collision_checker/scripts/.venv/bin/python -m ffw_peg_hole_env.server --port-base 7601
```

A working reference client is `tests/zmq_client_insertion.py` (scripted
insertion over the wire: 50/50 lockstep, 5/5 at 15 Hz).
