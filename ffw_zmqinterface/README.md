# ffw_zmqinterface — robot ↔ controller HIL link

ZeroMQ TCP link between the robot machine and an external controller (RL agent,
AI worker, HIL harness). The robot side binds three sockets; your controller
`connect()`s to all of them. This document defines the wire so either end can be
implemented independently. The canonical implementation lives in
`ffw_zmqinterface/protocol.py`; if you implement from this document alone, the
byte layout below is the contract.

## 30-second summary

| direction          | endpoint                 | socket / pattern   | messages                     |
|--------------------|--------------------------|--------------------|------------------------------|
| robot → controller | `tcp://<robot-ip>:6001`  | PUB                | `Obs` (EE + joints)          |
| robot → controller | `tcp://<robot-ip>:6003`  | PUB                | `Priv` (head-cam tf + deltas)|
| controller → robot | `tcp://<robot-ip>:6002`  | SUB                | `ControlCmd`, `OverrideCmd`  |

- Every ZMQ message is **one framed struct**: a fixed **12-byte header** + a
  **fixed-size payload**. ZMQ preserves message boundaries, so no extra framing.
- Units are **SI throughout**: meters, radians, seconds.
- End-effector (EE) indices: **`ee0` = right arm, `ee1` = left arm.**
- EE poses and goals (`Obs` EE block, `ControlCmd`) are expressed in the robot's
  **`map` frame**; the `Priv` head-camera matrix lands in the **control frame**
  (default `base_link`, see §2.5 and §3).
- **Message roles are split leRobot-style** so the controller can sync-and-concat:
  `Obs` is one complete, self-contained observation per tick (no cross-referencing
  a separate joint stream); `Priv` carries privileged context — head-cam tf and
  the per-tick human-commanded deltas — that is **not** fed to a learned network;
  `ControlCmd`/`OverrideCmd` are the action rail.
- **Naming note:** this ZMQ `Obs` (6001, EE + joint state) is **not** the same as
  the camera image "obs" stream a controller may separately receive (e.g. the
  SERL camera-server image Obs on port 7200). The two are unrelated rails.

---

## 1. The link

```
  Controller (your code)                    Robot machine
  ───────────────────────                   ───────────────────────────────
  ZMQ SUB connect :6001  ◄───────────────  gateway_node  PUB bind :6001  (Obs)
  ZMQ SUB connect :6003  ◄───────────────  gateway_node  PUB bind :6003  (Priv)
  ZMQ PUB connect :6002  ────────────────►  gateway_node  SUB bind :6002  (Control)
```

All robot sockets `bind`; the controller `connect`s. **Each PUB carries exactly
one message type** (6001 = `Obs` only, 6003 = `Priv` only), so there is no
cross-type CONFLATE hazard on the robot side and no header type dispatch needed
per socket — the sender leaves CONFLATE off and publishes every tick at full
rate. A consumer that only wants the **newest** frame of a rail may set
`ZMQ_CONFLATE=1` on its own SUB without starving anything; a consumer that wants
**every** frame (sync-and-concat logging, tf-batch pipelines) leaves CONFLATE off
and drains fully. (v1 published all robot→controller types on one socket, which
made per-socket CONFLATE unsafe; the split into 6001/6003 removes that.)

Controller read pattern: drain all pending frames each control cycle, keep the
newest `Obs` and the newest `Priv`, and act on those. Subscribing late simply
means you get no history; keep draining and the newest state arrives each cycle.

---

## 2. Wire protocol

### 2.1 Header (every message, 12 bytes, little-endian)

```
[0:4]    type      int32    -- message type id (see §2.2)
[4:12]   timestamp float64  -- seconds since epoch, sender clock
```

The type id dispatches the decoder (only needed if you multiplex sockets
yourself; the two PUB rails each carry one type). The timestamp is part of the
frame — it makes each message byte-unique even when the payload is unchanged, and
the robot side uses it for dedup and liveness (see §2.8).

### 2.2 Message types

| type | message        | direction                     | payload             | frame size |
|------|----------------|-------------------------------|---------------------|-----------|
| 1    | `Obs`          | robot → controller (PUB 6001) | 89 doubles = 712 B  | **724 B** |
| 2    | `ControlCmd`   | controller → robot            | 14 doubles = 112 B  | **124 B** |
| 3    | `Priv`         | robot → controller (PUB 6003) | 28 doubles = 224 B  | **236 B** |
| 4    | `OverrideCmd`  | controller → robot            | 25 doubles = 200 B  | **212 B** |

All payload values are `float64`, little-endian.

The v1 standalone messages are gone: `RobotState` (0), `JointState` (5), and
`EEDelta` (6) no longer exist — joints are folded into `Obs`, deltas into `Priv`.
`EEState` (1) and `HeadCamTf` (3) were the v1 names for the EE-block and
matrix-block halves that now live inside `Obs` and `Priv`; **their type ids are
reused** (Obs = 1, Priv = 3). `ControlCmd`(2) is byte-compatible with v1;
`OverrideCmd`(4) keeps the v1 id but is NOT byte-compatible -- its payload grew
from the v1 0/1 override latch (1 double) to the joint-space command rail (25
doubles, §2.6). Old EE/matrix consumers repointed at the new layout keep
working; a v1 override sender must reship to the new payload.

> **What you will actually see on a live link:** the gateway (`gateway_node`)
> publishes **`Obs` on 6001** and **`Priv` on 6003**, every gateway tick at the
> configured rate (default 100 Hz) once the underlying sources resolve (§4). The
> offline baseline `robot_node` also publishes `Obs` (synthetic sine-wave joints)
> so the two PUB rails are up without hardware. You send types 2 and 4 on 6002.

### 2.3 `Obs` — one complete observation (robot → controller, PUB 6001)

Payload: **89 doubles** -- joint block FIRST (index 0 == `arm_l_joint1`,
mirroring a recorded `ffw_il_recorder` state.csv row), then the EE + grippers
block at the tail:

```
[0:75]   Joint block  -- all 25 joints in JOINT_STATE_NAMES order (§2.3.1)
           [0:25]   joint_pos     (rad / m)
           [25:50]  joint_vel     (rad/s / m/s)
           [50:75]  joint_effort  (A)   -- measured motor current, not torque
[75:89]  EE block  -- dual EE poses + grippers (same 14-value order as ControlCmd)
           [75:81]  ee0 = (x, y, z, rx, ry, rz)   right arm, meters + radians
           [81]     grip0                         right gripper (0.0 open .. 1.0 close)
           [82:88]  ee1 = (x, y, z, rx, ry, rz)   left arm
           [88]     grip1                         left gripper (0.0 open .. 1.0 close)
```

The EE poses are the robot's current (achieved) poses in the `map` frame; the
gripper values are normalized from the current gripper joint angles with the
same open/closed bounds the solver commands with (command → feedback round-trip
symmetric: **0.0 = open, 1.0 = closed**). They read 0.0 until the gripper joints
are seen on `/joint_states`.

**This is the observation you log.** Because `Obs` is one self-contained frame
per tick (EE + grippers + all joints together), you can **sync and concat**:
record one `Obs` per tick and join them end-to-end into a trajectory — nothing
has to be cross-referenced against a separate joint or EE stream, and every
frame carries the sender clock in its header timestamp.

**Cadence:** the gateway emits `Obs` **every tick** at the configured rate
(default 100 Hz) once both arm EE poses are resolved **and** at least one
`/joint_states` has been seen. The joint block repeats the newest `/joint_states`
in every frame — the relay is **not** stamp-gated anymore, so the Obs cadence is
constant and a sync-and-concat logger gets exactly one frame per tick to append.
Frame headers are stamped fresh each send, so consecutive Obs frames differ even
when the payload repeats (a stuck source still ticks with constant cadence).

#### 2.3.1 Joint order (`JOINT_STATE_NAMES`)

The joint order is a fixed wire contract (`JOINT_STATE_NAMES` in `protocol.py`);
the gateway reorders incoming `/joint_states` into this order, filling 0.0 for
any joint missing from the message, so the layout is stable regardless of
broadcaster ordering. Index joints by name from this table:

| idx | name              | idx | name              |
|-----|-------------------|-----|-------------------|
| 0   | `arm_l_joint1`    | 13  | `arm_r_joint7`    |
| 1   | `arm_l_joint2`    | 14  | `gripper_l_joint1` |
| 2   | `arm_l_joint3`    | 15  | `gripper_r_joint1` |
| 3   | `arm_l_joint4`    | 16  | `head_joint1`     |
| 4   | `arm_l_joint5`    | 17  | `head_joint2`     |
| 5   | `arm_l_joint6`    | 18  | `left_wheel_drive` |
| 6   | `arm_l_joint7`    | 19  | `left_wheel_steer` |
| 7   | `arm_r_joint1`    | 20  | `lift_joint`      |
| 8   | `arm_r_joint2`    | 21  | `rear_wheel_drive` |
| 9   | `arm_r_joint3`    | 22  | `rear_wheel_steer` |
| 10  | `arm_r_joint4`    | 23  | `right_wheel_drive` |
| 11  | `arm_r_joint5`    | 24  | `right_wheel_steer` |
| 12  | `arm_r_joint6`    |     |                   |

Convenience constants in `protocol.py`: `GRIP_R_JOINT_IDX = 15`,
`GRIP_L_JOINT_IDX = 14` (the two gripper joints, right after the right-arm
chain; used by client-side
effort-based firm-grasp detection).

**Effort units:** the relay carries `/joint_states` effort as-is — for the
Dynamixel joints this is the measured **motor current (Present Current, in A)**,
**not** a controller-computed joint torque. The 3 base wheel drives run through
the `virtual_dxl` interface, so their effort may be 0.0 / low-fidelity. Treat it
as a load/current signal, not a torque.

### 2.4 `ControlCmd` — commanded end-effector poses + grippers (controller → robot)

Payload: 14 doubles:

```
[0:6]   ee0 = (x, y, z, rx, ry, rz)   right arm goal
[6]     grip0                         right arm gripper (0.0 open .. 1.0 close)
[7:13]  ee1 = (x, y, z, rx, ry, rz)   left arm goal
[13]    grip1                         left arm gripper (0.0 open .. 1.0 close)
```

Commands are **absolute goals in the `map` frame** — not deltas. The robot
solver walks toward the goal (leashed, roughly 6 cm per solver tick), so a
distant target is approached in steps; re-assert the goal each cycle to keep it
moving. If the controller goes silent for **0.5 s** (configurable), the robot
stops forwarding and holds its last goal. Values must be finite — non-finite
frames are dropped.

The gripper value is forwarded to the same trigger channel the physical
spacemouse quest trigger drives (the solver maps it to the gripper joints:
1 → close, 0 → open). **Gating caveat:** like the physical trigger, the gripper
feed is obeyed only while the quest is in **TRACK** — during CTRL/APPROACH it is
ignored, and a gripper value with no live controller (silent > 0.5 s) is not
re-applied until the controller resumes. The pose path is ungated; the gripper
path is TRACK-gated. Omit the gripper (or send 0.0) and the arm just moves with
the gripper left open.

### 2.5 `Priv` — privileged context: head-cam tf + commanded deltas (robot → controller, PUB 6003)

"Privileged" means **not fed to a learned network** — it is the context a
supervisor, demo logger, or vision pipeline needs but an agent should not be
trained on. Payload: **28 doubles**, two blocks:

```
[0:16]   HeadCamTf block  -- row-major 4x4 homogeneous matrix (16 doubles)
           [0:4]   row 0 = [m00 m01 m02 m03]
           [4:8]   row 1 = [m10 m11 m12 m13]
           [8:12]  row 2 = [m20 m21 m22 m23]
           [12:16] row 3 = [0   0   0   1   ]   (always the identity row)
[16:28]  EEDelta block
           [16:22] d0 = (dx, dy, dz, drx, dry, drz)   right arm commanded delta
           [22:28] d1 = (dx, dy, dz, drx, dry, drz)   left arm commanded delta
```

**Matrix — head camera → control frame.** The rotation `R` is the upper-left 3×3
(row-major); the translation is column 3 (`m03`, `m13`, `m23`). Mapping a
camera-space point into the control frame:

```
x_ctrl = m00*x_c + m01*y_c + m02*z_c + m03
y_ctrl = m10*x_c + m11*y_c + m12*z_c + m13
z_ctrl = m20*x_c + m21*y_c + m22*z_c + m23
```

i.e. `p_ctrl = M · [p_camera, 1]`. It is the robot-resolved
`lookup_transform(control_frame, head_camera_frame)` published by the
`head_camera_tf_bridge` node; the control frame is the frame the machine is
physically controlled in, **default `base_link`** (configurable there). So: detect
an object in the camera image, estimate `(x_c, y_c, z_c)` in camera coordinates,
apply this matrix, and the result is the object's position in the control frame —
no TF, no ROS, no frame juggling on your side. The transform resolves live and
reflects head motion; keep using the newest matrix.

**Deltas — per-tick commanded change.** What the spacemouse operator commanded
**this tick**, for each arm, in the `map` frame — the incremental change applied
to the arm's goal that tick, **after the soft-lock / limit-profile clamps**.
`dx,dy,dz` are meters; `drx,dry,drz` are radians (extrinsic-XYZ RPY, §2.10).
These pair with the `Obs` EE block: `Obs` is the pose every tick, the delta block
is the human's commanded change to it — exactly what the robot's solver recovers
by differencing consecutive `/spacemouse/<arm>/ee_target_pose` goals. This is
the expert label for demos, and the future action space if control ever moves to
delta space.

All-zeros means **"no command this tick"** — including the first tick after a
mapper re-base (arm switch, quest
engage/abort), which resets the baseline rather than commanding a jump. An arm
that is **not** spacemouse-driven right now goes silent and is **zero-filled** on
the wire. The deltas are held only while fresh (within a short window of the
spacemouse stream, default 0.25 s); older deltas are not repeated.

**Cadence:** the gateway emits `Priv` **every tick** once the head-cam transform
has resolved (before that, no `Priv` flows at all). The matrix repeats the newest
resolved transform; the delta block is per-tick or explicit zeros, so the frame
has a constant cadence even when nobody is commanding.

### 2.6 `OverrideCmd` — joint-space command rail (controller → robot)

Payload is the full commanded joint vector — 25 doubles in `JOINT_STATE_NAMES`
order, the **same order as the `Obs` joint block** (§2.3) and a dataset
q-column row:

```
[0:25]   joint_pos    commanded joint positions (rad / m)
         index N == Obs joint index N == dataset q-column index N
```

`OverrideCmd` is the **joint-space alternative to the EE-space `ControlCmd`**
rail: command the robot with either exact EE goals (`ControlCmd`) or the exact
25 joint positions in this order (`OverrideCmd`). It is **not a latch** — there
is no 0/1 engage bit; the command stream itself is the control signal, and the
receiver holds the newest frame and treats staleness like `ControlCmd`
(`cmd_timeout`, §2.4). Re-assert your full 25-joint vector at your control rate
(e.g. 25 Hz) exactly as you re-assert `ControlCmd`, stamping each frame with
the current time so the byte-dedup passes every re-assertion.

**Robot-side status:** `protocol.py` encodes/decodes this rail; `gateway_node`
decodes the newest frame and relays it to `/qpos_rail`
(`sensor_msgs/JointState`) every tick while fresh (same `cmd_timeout` gate as
`ControlCmd`). `ffw_ik_solver_teleop` consumes it (`goal_source=rail`,
`apply_rail_sync()`) and **rejects the whole message** unless it names every
joint its MuJoCo model expects with no unrecognized names (`gripper_l_joint1`
is the one allowed exception — it has no MuJoCo joint either). That model has
no mobile-base wheel joints at all, so the gateway drops the 6
`*_wheel_drive`/`*_wheel_steer` names from the 25 before forwarding and relays
every other joint (arms, grippers, head, lift) by name. The v1 0/1 latch
semantics and the gateway's `/control_override` `std_msgs/Bool` relay are
**gone**; `joy_hand`'s `/control_override` force-TRACK coupling is retired
with it.

### 2.7 Removed v1 messages

- `RobotState` (type 0, baseline 20-joint sine) — **removed**. The offline
  baseline now publishes the same `Obs` shape the gateway does.
- `JointState` (type 5) — **folded into `Obs`** (§2.3 joint block). No standalone
  joint rail anymore.
- `EEDelta` (type 6) — **folded into `Priv`** (§2.5 delta block).

### 2.8 The timestamp contract (important for the controller)

Every frame carries a `float64` sender-clock timestamp in the header. The robot
side byte-dedups inbound `ControlCmd`: **two frames with identical
(type, timestamp, payload) are treated as one retransmission** and only the
first is forwarded. This means:

- To re-assert the same goal (which you normally do every cycle to walk the
  leash), stamp each send with the **current time** → every frame passes.
- Reusing a stale/identical timestamp for a re-send → that frame is swallowed
  (stuck-sender guard).

So: **always timestamp your sends with `time.time()`.** A fresh timestamp per
send is free; a reused one silently drops your command.

### 2.9 Decoder validation

Receivers validate the frame before decoding and drop it on mismatch:

- total length ≠ expected frame size for the declared type → error;
- header type id ≠ the type the decoder expects → error.

`Obs` (89 doubles) and `Priv` (28 doubles) have distinct payload sizes and type
ids. `ControlCmd` (14 doubles) is laid out like the `Obs` EE tail block
(ee0/grip0/ee1/grip1, same value order) but is a distinct message -- always
dispatch on the header type id, never on payload size alone.

### 2.10 RPY / quaternion convention

Orientations are encoded as **RPY in radians, extrinsic XYZ / intrinsic ZYX**:

```
R = Rx(roll) · Ry(pitch) · Rz(yaw)
```

This is the convention used by the robot's teleop stack. If you work in
quaternions, convert with the standard formulas implemented in
`protocol.py::rpy_to_quat` / `quat_to_rpy` (they use exactly this convention) —
or copy `protocol.py` wholesale.

---

## 3. Typical use: vision → grab

1. Receive video frames on the **separate UDP/video stream** (this ZMQ link does
   *not* carry pixels — RGB is streamed separately, e.g. a camera-server Obs
   image rail on its own port; see the naming note in the summary).
2. Detect the object and estimate its 3-D position in **camera coordinates**
   `(x_c, y_c, z_c)` using the camera intrinsics.
3. Read the newest `Priv` frame (6003); compute `p_ctrl = M · [p_camera, 1]` —
   the object's position in the **control frame** (default `base_link`).
4. Read the newest `Obs` frame (6001) for the current EE poses and send
   `ControlCmd` goals. Note the frame split: the matrix output is the control
   frame (default `base_link`), while EE poses/goals on the wire are in the
   robot's `map` frame; the `map`↔`base_link` relationship is the robot's
   localization, and the two coincide when the robot is at the map origin. Plan
   your goal in whichever frame you prefer and convert if you mix the two.
5. Send the goal as a `ControlCmd` on 6002 with a fresh timestamp, re-asserting
   each cycle until the pose is reached.
6. To grab, set the arm's gripper value in the same `ControlCmd` (0.0 open ..
   1.0 close) and re-assert it together with the goal; the feedback comes back
   in the `Obs` EE block. Remember the gripper is obeyed only while the quest is
   in TRACK (see §2.4).

---

## 4. Rates / liveness (defaults)

| stream           | default rate | notes |
|------------------|--------------|-------|
| `Obs` (6001)     | 100 Hz       | gateway `hz` parameter; starts once both EE poses are seen AND the first `/joint_states` arrives; the joint block repeats the newest `/joint_states`, so cadence is constant |
| `Priv` (6003)    | 100 Hz       | gateway tick; starts once the head-cam transform resolves; matrix repeats the newest resolved transform (~30 Hz bridge `update_rate`), delta block is per-tick or explicit zeros |
| `ControlCmd`     | yours        | send as fast as you want to re-assert goals (≤ ~100 Hz) |
| `OverrideCmd`    | yours        | re-assert the full 25-joint command vector at your control rate whenever joint-space control is active |

Each robot→controller rail carries one type on one socket, so nothing needs
dispatch by header type id — but every frame still carries it (and the decoder
validates it) so sockets can be shared or logs replayed later.

---

## 5. Minimum controller implementation

`protocol.py` in this package is the reference and is deliberately dependency-
free (pure stdlib: `struct`, `time`). Two ways to consume it:

**A. Copy `protocol.py`** (recommended). Drop it into your project and use
`decode_obs`, `decode_priv`, `encode_control`, `encode_override`, etc. The
module docstring documents every layout. Only the encoders/decoders need to
match across the link.

**B. Implement from this document.** The two things you need:

```python
import struct, time
import zmq

HDR = struct.Struct("<id")            # int32 type + float64 timestamp

OBS = zmq.Context().socket(zmq.SUB)   # robot -> you: Obs only
OBS.connect("tcp://<robot-ip>:6001")
OBS.setsockopt(zmq.SUBSCRIBE, b"")

PRIV = zmq.Context().socket(zmq.SUB)  # robot -> you: Priv only
PRIV.connect("tcp://<robot-ip>:6003")
PRIV.setsockopt(zmq.SUBSCRIBE, b"")

PUB = zmq.Context().socket(zmq.PUB)   # you -> robot
PUB.connect("tcp://<robot-ip>:6002")

def recv(sock):
    frame = sock.recv()
    t, ts = HDR.unpack_from(frame, 0)
    vals = struct.unpack(f"<{len(frame) - 12}d", frame[12:])
    return t, ts, vals

def send_goal(ee0, ee1, grip=(0.0, 0.0)):  # grip = (grip0, grip1) 0=open 1=close
    pub_frame = HDR.pack(2, time.time()) + struct.pack(
        "<14d", *(ee0 + (grip[0],) + ee1 + (grip[1],)))
    PUB.send(pub_frame)

def send_override(qpos):                    # joint-space rail: 25 qpos in JOINT_STATE_NAMES order
    assert len(qpos) == 25
    PUB.send(HDR.pack(4, time.time()) + struct.pack("<25d", *qpos))

# poll() both SUBs so a slow rail never blocks the fast one
poller = zmq.Poller()
poller.register(OBS, zmq.POLLIN)
poller.register(PRIV, zmq.POLLIN)

while True:
    for sock, _ in poller.poll(0):          # non-blocking: act on newest only
        t, ts, v = recv(sock)               # 6001 -> t == 1 (Obs); 6003 -> t == 3 (Priv)
        if t == 1:
            # Obs (89 doubles): v[0:25]=pos, v[25:50]=vel, v[50:75]=effort --
            #   25 joints in JOINT_STATE_NAMES order (see §2.3.1); EE at the
            #   tail: v[75:81]=right ee, v[81]=right grip, v[82:88]=left ee,
            #   v[88]=left grip
            pass
        elif t == 3:
            # Priv (28 doubles): v[0:16]=row-major 4x4 head-cam->control matrix,
            #   v[16:22]=right delta, v[22:28]=left delta (all-zeros = no command)
            pass
```

Runtime requirements: a ZMQ binding (e.g. `pyzmq`). No ROS is needed on the
controller side.

---

## 6. Robot-side nodes (context, not required reading)

- **`gateway_node`** — the live link. Publishes **`Obs`** (6001) from the
  solver's achieved EE poses (map frame) + the grippers and all joints
  normalized/reordered from `/joint_states`, every tick once both EE poses and
  the first joint message are seen (§2.3); publishes **`Priv`** (6003) carrying
  the resolved `/head_camera_tf` as the matrix and the per-tick spacemouse
  commanded deltas (`/spacemouse/<arm>/ee_target_delta`) as the delta block,
  zero-filling an arm that is not spacemouse-driven (§2.5); receives
  `ControlCmd` and forwards each frame as an absolute map-frame goal to the arm
  solver (`/quest/<arm>/ee_target_pose`) plus the gripper value to
  `/quest/<arm>/trigger`; decodes `OverrideCmd` and relays it to `/qpos_rail`
  (`sensor_msgs/JointState`), dropping the 6 wheel-drive/steer names the
  solver's model doesn't recognize and forwarding every other joint by name
  (§2.6). Dedup and `cmd_timeout` behave as described in §2.8 / §2.4.
- **`robot_node`** — baseline stand-in for bring-up/testing with no hardware:
  publishes synthetic sine-wave `Obs` (same shape as the gateway's) and drains
  `ControlCmd`. Not part of the live bring-up.

## 7. Test

```
python3 -m pytest test/
```

Covers round-trips for all four types, wire sizes (724/124/236/212 B), the Obs
joint block leading (EE + grippers block at the tail), length and type-mismatch
rejection, gripper clamping, timestamp uniqueness, the `JOINT_STATE_NAMES`/gripper
index contract, the `OverrideCmd`-order contract, and the RPY/quat conversions.
Uses only stdlib + pytest.

## 8. Examples

`examples/` holds robot-side dev tools that drive the arm through this link,
not part of the controller contract. The earlier live scripts
(`rectangle_sweep.py`, `release_cube.py`) gated on the v1 `/control_override`
Bool latch and are now **retired** — they live frozen under
`examples/deprecated/` (see the banner in each file and `examples/README.md`).
Do not run them on a live link, and do not re-add `/control_override` to bring
them back.
