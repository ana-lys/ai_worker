# ffw_peg_hole_env

> **Real robot:** see `REAL_ROBOT_README.md` (server, wire `Frame`, episode rules, offset
> calibration, safety, remaining plan). This file describes the parked MuJoCo sim server.

Server side of the peg-in-hole task for HIL-SERL: the functions that run when
HIL-SERL's ZMQ messages arrive. One server, the same event handling for the
MuJoCo simulator and (next) the real robot; only the timing differs.

No ROS, no gym. Runs in `ffw_collision_checker/scripts/.venv` (mujoco 3.13,
numpy, scipy, zmq).

## Run

```bash
cd ~/robotis_ws/src/ai_worker/ffw_peg_hole_env
PY=../ffw_collision_checker/scripts/.venv/bin/python

$PY -m ffw_peg_hole_env.server --port-base 7601            # sim, lockstep (as fast as commands arrive)
$PY -m ffw_peg_hole_env.server --port-base 7601 --realtime # sim, 15 Hz wall clock
#   --timestep 0.004 (default)   --no-cameras   --view (MuJoCo viewer, needs MUJOCO_GL=glfw)

$PY tests/test_wire_and_task.py                 # unit checks
$PY tests/run_sim_insertion.py --episodes 100   # in-process, scripted insertion
$PY tests/zmq_client_insertion.py --episodes 30 # end to end over ZMQ (starts its own server)
```

Rendering is headless EGL on GPU device 0 by default (`MUJOCO_GL=egl`,
`MUJOCO_EGL_DEVICE_ID=0`); set `MUJOCO_GL=glfw` for on-screen windows.

The robot gateway on this workstation already binds 6001–6004, so run the sim
server on another port base until the two are merged.

## Ports (base + n)

| port | socket | content |
|---|---|---|
| base + 0 | PUB | `Obs`, the robot gateway's frame (`ffw_zmqinterface/protocol.py`, 131 doubles), after every tick and after a reset |
| base + 1 | SUB | `ControlCmdDelta`, `EnvCmd` |
| base + 4 | PUB | `EnvStatus` after every tick and every state change |
| base + 5 | PUB | wrist images, multipart, right after the `Obs` of the same tick |

base + 2 (`Priv`) and base + 3 (`Record`) stay the gateway's. Offsets live in
`wire.PORT_*`; the full Obs layout is in `wire.py`'s docstring.

## Messages (`ffw_peg_hole_env/wire.py`)

All frames start with the gateway's 12-byte header: int32 type, float64 timestamp.

| type | name | payload |
|---|---|---|
| 6 | `ControlCmdDelta` | 14 float64: right (dx, dy, dz, droll, dpitch, dyaw, grip), left (same) |
| 8 | `EnvCmd` | cmd uint8 (1 RESET, 2 PAUSE, 3 RESUME, 4 ABORT, 5 PING), seed int64, episode_id uint32, 8 float64 reserved |
| 9 | `EnvStatus` | state, backend, reset_ok, success, terminated, truncated (uint8); episode_id, step, deltas_received (uint32); reward, depth, lateral, contact_force (float64) |
| 10 | `Images` | multipart `[header + episode_id, step (uint32), h, w (uint16), right RGB, left RGB]` |

States: 0 IDLE, 1 RESETTING, 2 READY, 3 RUNNING, 4 PAUSED, 5 DONE, 6 FAULT.

Agreed with the HIL-SERL side (2026-10-02): `ControlCmdDelta` stays 6 and the
gateway's `SetMode` moves; types 8 / 9 / 10 accepted; status and images on
their own ports.

## Episode

1. Send `EnvCmd RESET(seed, episode_id)`. The server samples, in the hole frame
   (z = insertion axis, up): hole x/y ±5 cm, z 0 → −2 cm; peg x/y ±5 cm, tip
   2–4 cm above the rim, ±5° tilt. Only samples whose insertion is reachable
   are used. Reply: `EnvStatus READY` (or `FAULT`, reset_ok = 0), then `Obs`
   and images for step 0.
2. Each `ControlCmdDelta` → one tick → `Obs`, images, `EnvStatus` with the new
   step. In the 15 Hz mode the deltas received during a tick are composed; no
   delta = hold.
3. `EnvStatus.terminated`: success (peg tip ≥ 34 mm below the rim — the robot's full push — and within
   2 mm of the hole axis) or failure (push-back on the peg > 10 N, or > 8 N within 6 mm of the rim, see Reward;
   the sim also fails on true contact > 300 N). `truncated`: 300 steps (20 s).
   After either, the server is DONE and ignores deltas until the next RESET.

## Reward (`ffw_peg_hole_env/reward.py`, `RewardConfig`)

Computed only from what both backends observe (Obs amps, joint state, the
peg/hole poses), so sim and robot score the same way. Per step:

| term | value |
|---|---|
| success | once (terminal), tip ≥ 34 mm in and ≤ 2 mm off the axis: **1 + 0.5 · (1 − F_peak / 10 N)**, floored at 1. F_peak = highest push-back since the peg could first touch the hole: 0 N → 1.5, 2 N → 1.4, 4 N → 1.3, 10 N → 1.0 |
| shape | potential-based progress φ′ − φ, φ = −0.3 · mean(lateral/5 cm, tilt/5°, depth-to-go/7.4 cm), each capped at 1; depth below the rim only counts on the hole axis. At most +0.3 per episode |
| force | −0.01 · clip(F_pb / 10 N, 0, 1) every step, no dead zone |
| fail | once (terminal), only while the peg can touch the hole: F_pb > 10 N (jam; a stuck peg that keeps being pushed ends here), or > 8 N for 2 ticks with the tip ≤ 6 mm in (rim strike). **−0.5 − 0.5 · share of the 34 mm not reached**: jam at 33 mm ≈ −0.5, stuck at 17 mm ≈ −0.75, rim ≈ −1 |

F_pb = push-back on the peg along its axis from the right arm's currents:
amps → joint torque with the load-cell driving K (K · gear efficiency,
`effort.py`), → force by weighted LS through Jᵀ at `right_peg_site` (the
`peg_hole_teach.EdgeForce` math), minus a reference held while the peg cannot
touch the hole. It reads like the teach GUI's force numbers. `EnvStatus.contact_force`
carries F_pb (not the sim's true contact), so the client sees what the robot
will report. In-process, `step()` returns every term in `info["reward_terms"]`.

## Client contract (what the HIL-SERL env must do)

- **Delta semantics.** Applied to the commanded **IK end-effector** pose
  (`*_gripper_site`, the pose `Obs.ee` reports): translation added in
  base_link; rotation R_new = R_delta · R with R_delta from intrinsic X-Y-Z
  angles, the same as the `Obs` rpy convention (R = Rx·Ry·Rz).
- **Clip like the server.** The server clips each delta **per axis**: each
  translation component to ±6.67 mm and each rotation component to ±2° per
  tick (agent action ±1 on every axis = the cap; at 15 Hz that is up to
  10 cm/s per axis, 17 cm/s diagonally, 30°/s per rotation axis). Clip before sending, so the delta sent is the delta applied; otherwise
  a client that tracks the commanded pose drifts away from the server's.
- **Latch at READY.** The command reference is reset at every RESET; take the
  commanded pose from the step-0 `Obs`.
- **Steer from the observed pose.** Compute deltas from `Obs.ee`, not only from
  your own commanded pose, so tracking offsets are corrected.
- **grip is ignored**: the peg and hole stay gripped (sim gripper fixed at the
  robot's 2026-10-01 angles, reported in `Obs.gripper` and the joint block).
- **Effort** in `Obs` is amps on every arm joint, on both backends: the robot
  gateway converts the driver's raw `/joint_states` effort (YM080 ÷13, YM070
  ÷25 in 0.01 A counts, PH42 J7 in mA); the sim reports actuator torque through
  the inverse of the same load-cell driving K (`effort.py`). Other
  joints stay in driver units. The marker-frame block is left at its defaults
  in sim.
- **Images**: pair them with the status by (episode_id, step).

## Measured (RTX 4060 Ti workstation)

| setup | steps/s |
|---|---|
| in-process, no cameras (4 ms physics) | ~1200 |
| in-process, two 128×128 cameras | ~400 |
| over ZMQ, lockstep, with images, incl. resets | ~240 |
| over ZMQ, 15 Hz mode | 15 |
