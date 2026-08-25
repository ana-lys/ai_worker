# ffw_zmqinterface

Robot-side ZMQ HIL (hardware-in-the-loop) link. The node runs on the machine
attached to the robot and talks to the RL/controller side over ZMQ TCP.

The wire carries **framed structs**: every ZMQ message is a fixed 12-byte header
followed by the payload. ZMQ preserves message boundaries, so the header is the
only framing needed. `ffw_zmqinterface/protocol.py` is the reference
implementation (encoders/decoders + byte layout); the layout below is what the
other end of the link must match.

## Link

| direction        | socket | port  | pattern                | message      |
|------------------|--------|-------|------------------------|--------------|
| robot -> control | PUB    | 6001  | latest-wins (CONFLATE) | `RobotState` or `EEState` |
| control -> robot | SUB    | 6002  | keep newest (CONFLATE) | `ControlCmd` |

Both sockets `bind` on the robot machine; the controller `connect`s. `robot_node`
and `gateway_node` are mutually exclusive (both bind 6001/6002).

## Wire protocol

One message == one frame:

```
[0:4]    type      (int32,  little-endian)   -- message type id, see below
[4:12]   timestamp (float64, little-endian)  -- seconds since epoch, sender clock
[12:12+len] payload (little-endian doubles)
```

| type | message       | payload                     | frame size |
|------|---------------|-----------------------------|-----------|
| 0    | `RobotState`  | 60 doubles = 480 B          | **492 B**  |
| 1    | `EEState`     | 12 doubles = 96 B           | **108 B**  |
| 2    | `ControlCmd`  | 12 doubles = 96 B           | **108 B**  |

Payload layouts:

- **`RobotState`** (robot -> controller): 20 joints x (pos, vel, acc).
  `[0:20]` joint_pos (rad / m), `[20:40]` joint_vel (rad/s / m/s),
  `[40:60]` joint_acc (rad/s^2 / m/s^2).
- **`EEState`** (gateway -> controller): two 6-DOF poses, `ee0` + `ee1`.
- **`ControlCmd`** (controller -> robot): two 6-DOF target poses, `ee0` + `ee1`.

Each 6-DOF pose is `(x, y, z, rx, ry, rz)` — position in **meters**, orientation
as **RPY in radians, extrinsic XYZ** (`R = Rx(roll) * Ry(pitch) * Rz(yaw)`),
matching the spacemouse teleop stack. `ee0` is the **right** arm, `ee1` the
**left** arm. Units are SI throughout.

### The header timestamp

The timestamp is part of the frame, so two messages with the same payload but
different timestamps are **different bytes**. The gateway byte-dedup drops only
an exact retransmission (same type + ts + payload):

- Controller re-asserting the same goal at 25 Hz with a **fresh** timestamp per
  send -> every frame passes the dedup. This is what walks the solver's leash.
- Controller reusing the **same** timestamp -> frames are byte-identical and
  only the first is forwarded (stuck-sender guard).

So the controller must stamp every send with the current send time. The
reference `encode_*` functions auto-stamp with `time.time()` when `ts=None`;
passing an explicit stale `ts` reproduces the stuck-sender behavior.

### Decoder rules

Receivers validate length and type and drop the frame on mismatch:
- wrong total length for the declared type -> `ValueError`
- header type id != the type the decoder expects -> `ValueError`
- `EEState` and `ControlCmd` share the 96-B payload but have distinct type ids,
  so a receiver cannot mistake one for the other (the old "reuse the ControlCmd
  decoder" shortcut is gone).

## Node: `gateway_node` (live robot link)

Bridges the ZMQ link to the spacemouse teleop stack on the robot machine:

- subscribes `/ik_solver/achieved_ee_pose_r` / `..._l` (achieved EE poses, map
  frame) and publishes them as **one `EEState`** on 6001 every tick (both arms;
  publishes only after both are seen). Position comes through directly;
  orientation is converted quat -> RPY for the wire.
- subscribes 6002 and forwards the newest `ControlCmd` as absolute goals on
  `/quest/right/ee_target_pose` / `/quest/left/ee_target_pose` (map frame,
  `PoseStamped`, RPY -> quat). Forwarded frames are byte-deduped as above.
- drops frames with non-finite values (with a warning) and malformed frames.
- `cmd_timeout` (default **0.5 s**): stops forwarding once no `ControlCmd` has
  arrived for that long. The solver then holds its last quest goal until the
  spacemouse moves again.

The `/quest/<arm>/ee_target_pose` path consumes an absolute map-frame goal
directly, leashed ~6 cm/tick, and is NOT gated on the quest being active — the
correct injection point for remote commands. The delta path
`/spacemouse/<arm>/ee_target_pose` treats a large jump as a mapper re-base, so
it is not used here.

Parameters:

```
ros2 run ffw_zmqinterface gateway_node --ros-args \
    -p state_port:=6001 -p control_port:=6002 -p hz:=100.0 -p cmd_timeout:=0.5
```

The ee-index <-> arm mapping is `ee0 -> right, ee1 -> left`; to swap, flip the
`_CMD_TOPIC` / `_ACHIEVED_TOPIC` dicts in `gateway_node.py`.

## Node: `robot_node` (baseline / no-ROS)

Publishes a synthetic sine-wave `RobotState` on 6001 and drains `ControlCmd`
from 6002 (keeps the newest), so the link works with no hardware or ROS.
Replace `build_state()` with real joint reads (e.g. `/joint_states`) to go live.

```
python3 ffw_zmqinterface/robot_node.py [--state-port 6001] [--control-port 6002] [--hz 100]
```

## Controller-side (the other machine)

Implement or copy the framed wire above. The safest path: copy
`ffw_zmqinterface/protocol.py` over your controller's `protocol.py` (e.g. SERL
`hil-serl`) — the encoders/decoders are the only thing that must match across
the link. If reimplementing, honor exactly: little-endian, the `<id` header,
the three type ids, the frame sizes (492 / 108 / 108 B), and the extrinsic-XYZ
RPY convention. The reference `rpy_to_quat` / `quat_to_rpy` helpers use that
convention and are provided for converting to/from quaternions on either end.

Runtime requirements on the robot machine: `python3-zmq`, `rclpy`,
`geometry_msgs`. The controller side needs only a ZMQ lib (no ROS).

## Test

```
python3 -m pytest test/
```

pyzmq (`python3-zmq`) is required at runtime but not for the protocol tests
(`test/test_protocol.py` covers round-trips, wire sizes, type/length rejection,
timestamp-uniqueness, and the RPY/quat conversions).
