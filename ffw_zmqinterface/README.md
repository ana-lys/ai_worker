# ffw_zmqinterface — robot ↔ controller HIL link

ZeroMQ TCP link between the robot machine and an external controller (RL agent,
AI worker, HIL harness). The robot side binds two sockets; your controller
`connect()`s to both. This document defines the wire so either end can be
implemented independently. The canonical implementation lives in
`ffw_zmqinterface/protocol.py`; if you implement from this document alone, the
byte layout below is the contract.

## 30-second summary

| direction          | endpoint                 | socket / pattern   | messages |
|--------------------|--------------------------|--------------------|----------|
| robot → controller | `tcp://<robot-ip>:6001`  | PUB, latest-wins (CONFLATE) | `EEState`, `HeadCamTf` |
| controller → robot | `tcp://<robot-ip>:6002`  | SUB, keep-newest (CONFLATE) | `ControlCmd` |

- Every ZMQ message is **one framed struct**: a fixed **12-byte header** + a
  **fixed-size payload**. ZMQ preserves message boundaries, so no extra framing.
- Units are **SI throughout**: meters, radians, seconds.
- End-effector (EE) indices: **`ee0` = right arm, `ee1` = left arm.**
- EE poses and goals (`EEState`, `ControlCmd`) are expressed in the robot's
  **`map` frame**; the `HeadCamTf` transform lands in the robot-fixed
  **`base_link`** frame (see §3).

---

## 1. The link

```
  Controller (your code)                    Robot machine
  ───────────────────────                   ───────────────────────────────
  ZMQ SUB connect :6001  ◄────────────────  gateway_node  PUB bind :6001
  ZMQ PUB connect :6002  ────────────────►  gateway_node  SUB bind :6002
```

Both robot sockets `bind`; the controller `connect`s. `CONFLATE=1` means the
robot side keeps only the newest outbound message and newest inbound message —
subscribing late or missing ticks is fine; you always get the latest state.

---

## 2. Wire protocol

### 2.1 Header (every message, 12 bytes, little-endian)

```
[0:4]    type      int32    -- message type id (see §2.2)
[4:12]   timestamp float64  -- seconds since epoch, sender clock
```

The type id dispatches the decoder. The timestamp is part of the frame — it
makes each message byte-unique even when the payload is unchanged, and the robot
side uses it for dedup and liveness (see §2.7).

### 2.2 Message types

| type | message        | direction                     | payload            | frame size |
|------|----------------|-------------------------------|--------------------|-----------|
| 0    | `RobotState`   | robot → controller (baseline node only) | 60 doubles = 480 B | **492 B** |
| 1    | `EEState`      | robot → controller            | 12 doubles = 96 B  | **108 B** |
| 2    | `ControlCmd`   | controller → robot            | 12 doubles = 96 B  | **108 B** |
| 3    | `HeadCamTf`    | robot → controller            | 16 doubles = 128 B | **140 B** |

All payload values are `float64`, little-endian.

> **What you will actually see on a live link:** the gateway (`gateway_node`)
> publishes **`EEState`** and **`HeadCamTf`** on 6001. `RobotState` is emitted
> only by the baseline `robot_node`, which is an offline stand-in, not part of
> the live bring-up — if you connect to the robot you can rely on types 1 and 3
> flowing, and you send type 2.

### 2.3 `EEState` — current end-effector poses (robot → controller)

Payload: two 6-DOF poses, 12 doubles:

```
[0:6]   ee0 = (x, y, z, rx, ry, rz)   right arm, in meters + radians
[6:12]  ee1 = (x, y, z, rx, ry, rz)   left arm
```

These are the robot's current (achieved) EE poses in the `map` frame.

### 2.4 `ControlCmd` — commanded end-effector poses (controller → robot)

Identical layout to `EEState`:

```
[0:6]   ee0 = (x, y, z, rx, ry, rz)   right arm goal
[6:12]  ee1 = (x, y, z, rx, ry, rz)   left arm goal
```

Commands are **absolute goals in the `map` frame** — not deltas. The robot
solver walks toward the goal (leashed, roughly 6 cm per solver tick), so a
distant target is approached in steps; re-assert the goal each cycle to keep it
moving. If the controller goes silent for **0.5 s** (configurable), the robot
stops forwarding and holds its last goal. Values must be finite — non-finite
frames are dropped.

### 2.5 `HeadCamTf` — head-camera → robot base_link transform (robot → controller)

The transform that converts a detection in **head-camera coordinates** into the
robot's **`base_link` frame** — the robot-fixed frame the machine is physically
controlled in. Payload is a 4×4 homogeneous matrix, **row-major**, 16 doubles:

```
[0:4]   row 0 = [m00 m01 m02 m03]
[4:8]   row 1 = [m10 m11 m12 m13]
[8:12]  row 2 = [m20 m21 m22 m23]
[12:16] row 3 = [0   0   0   1   ]   (always the identity row)
```

The rotation `R` is the upper-left 3×3 (row-major); the translation is column 3
(`m03`, `m13`, `m23`). Mapping a camera-space point to a base_link-space point:

```
x_base = m00*x_c + m01*y_c + m02*z_c + m03
y_base = m10*x_c + m11*y_c + m12*z_c + m13
z_base = m20*x_c + m21*y_c + m22*z_c + m23
```

i.e. `p_base = M · [p_camera, 1]`. So: detect an object in the camera image,
estimate `(x_c, y_c, z_c)` in camera coordinates, apply this matrix, and the
result is the object's position in the robot `base_link` frame — no TF, no ROS,
no frame juggling on your side. (`EEState`/`ControlCmd` are expressed in the
`map` frame; see §3.)

The transform is resolved live on the robot from the current head pose
(`base_link` → `head_camera_frame` needs only the robot model + head joint
states), so it reflects head motion; keep using the newest frame.

### 2.6 `RobotState` — 20 joint state (baseline node only)

Payload: 60 doubles, 20 joints × (pos, vel, acc):

```
[0:20]   joint_pos (rad / m)
[20:40]  joint_vel (rad/s / m/s)
[40:60]  joint_acc (rad/s² / m/s²)
```

Emitted by the offline `robot_node` baseline only (synthetic sine waves).

### 2.7 The timestamp contract (important for the controller)

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

### 2.8 Decoder validation

Receivers validate the frame before decoding and drop it on mismatch:

- total length ≠ expected frame size for the declared type → error;
- header type id ≠ the type the decoder expects → error.

`EEState` and `ControlCmd` share the 96-B payload but have **distinct type ids**,
so they cannot be confused. Always dispatch on the header type id, never on
payload size alone.

### 2.9 RPY / quaternion convention

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

1. Receive video frames on the **separate UDP video stream** (this ZMQ link does
   *not* carry pixels — RGB is streamed separately, e.g. port 9000).
2. Detect the object and estimate its 3-D position in **camera coordinates**
   `(x_c, y_c, z_c)` using the camera intrinsics.
3. Read the newest `HeadCamTf` frame; compute `p_base = M · [p_camera, 1]` —
   the object's position in the robot `base_link` frame.
4. Read the newest `EEState` for the current EE poses and send `ControlCmd`
   goals. Note the frame split: the transform output is `base_link`, while EE
   poses/goals on the wire are in the robot's `map` frame; the `map`↔`base_link`
   relationship is the robot's localization, and the two coincide when the robot
   is at the map origin. Plan your goal in whichever frame you prefer and
   convert if you mix the two.
5. Send the goal as a `ControlCmd` on 6002 with a fresh timestamp, re-asserting
   each cycle until the pose is reached.

---

## 4. Rates / liveness (defaults)

| stream          | default rate | notes |
|-----------------|--------------|-------|
| `EEState`       | 100 Hz       | gateway `hz` parameter; published once both EE poses are seen |
| `HeadCamTf`     | 30 Hz        | bridge `update_rate`; published once the robot-side transform resolves |
| `ControlCmd`    | yours        | send as fast as you want to re-assert goals (≤ ~100 Hz) |

Nothing on the wire is guaranteed to arrive in a fixed order — dispatch on the
header type id. `EEState` and `HeadCamTf` are independent messages on the same
socket.

---

## 5. Minimum controller implementation

`protocol.py` in this package is the reference and is deliberately dependency-
free (pure stdlib: `struct`, `time`). Two ways to consume it:

**A. Copy `protocol.py`** (recommended). Drop it into your project and use
`decode_ee_state`, `decode_head_cam_tf`, `encode_control`, etc. The module
docstring documents every layout. Only the encoders/decoders need to match
across the link.

**B. Implement from this document.** The two things you need:

```python
import struct, time
import zmq

HDR = struct.Struct("<id")            # int32 type + float64 timestamp

SUB = zmq.Context().socket(zmq.SUB)   # robot -> you
SUB.connect("tcp://<robot-ip>:6001")
SUB.setsockopt(zmq.SUBSCRIBE, b"")

PUB = zmq.Context().socket(zmq.PUB)   # you -> robot
PUB.connect("tcp://<robot-ip>:6002")

def recv():
    frame = SUB.recv()
    t, ts = HDR.unpack_from(frame, 0)
    vals = struct.unpack(f"<{len(frame) - 12}d", frame[12:])
    return t, ts, vals

def send_goal(ee0, ee1):              # ee0=(x,y,z,rx,ry,rz), ee1 likewise
    pub_frame = HDR.pack(2, time.time()) + struct.pack("<12d", *(ee0 + ee1))
    PUB.send(pub_frame)

while True:
    t, ts, v = recv()
    if t == 1:      # EEState: v[0:6]=right, v[6:12]=left
        pass
    elif t == 3:    # HeadCamTf: 16 doubles, row-major 4x4
        pass
    elif t == 0:    # RobotState: 60 doubles (baseline only)
        pass
```

Runtime requirements: a ZMQ binding (e.g. `pyzmq`). No ROS is needed on the
controller side.

---

## 6. Robot-side nodes (context, not required reading)

- **`gateway_node`** — the live link. Subscribes the solver's achieved EE poses
  (map frame) and publishes them as `EEState`; subscribes `/head_camera_tf`
  (a ROS topic carrying the resolved camera→base_link transform) and
  republishes it as `HeadCamTf`; receives `ControlCmd` and forwards each frame
  as an absolute map-frame goal to the arm solver
  (`/quest/<arm>/ee_target_pose`). Dedup and `cmd_timeout` behave as described
  in §2.7 / §2.4.
- **`robot_node`** — baseline stand-in for bring-up/testing with no hardware:
  publishes synthetic sine-wave `RobotState`, drains `ControlCmd`. Not part of
  the live bring-up.

## 7. Test

```
python3 -m pytest test/
```

Covers round-trips for all four types, wire sizes (492/108/108/140 B), length
and type-mismatch rejection, timestamp uniqueness, and the RPY/quat
conversions. Uses only stdlib + pytest.

## 8. Examples

`examples/` holds live-robot demos that drive the arm through this link —
see `examples/README.md`. `rectangle_sweep.py` moves the right EE around a
rectangle via `ControlCmd` re-assertions (the "move the arm in a square" demo).
These are robot-side dev tools, not part of the controller contract; they need
ROS + the running stack and must only be run against a live robot deliberately.
