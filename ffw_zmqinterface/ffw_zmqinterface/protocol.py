"""Wire protocol for the ffw_zmqinterface HIL link.

One ZMQ message == one framed struct: a fixed 12-byte header followed by
the message payload. ZMQ preserves message boundaries, so the header is the
only framing needed.

Header (every message):
    [0:4]    type      (int32, little-endian)  -- see MSG_* below
    [4:12]   timestamp (float64, seconds since epoch, sender clock)

The type id lets the receiver dispatch to the correct decoder (the old
"same wire layout, reuse the ControlCmd decoder" hack is gone). The
timestamp makes every frame unique on the wire even when the payload is
identical -- e.g. a controller re-asserting the same EE goal at 25 Hz gets
each re-assertion through a byte-dedup because the bytes differ.

Message roles (leRobot-style separation):

  Obs     (robot -> controller, PUB on tcp://*:STATE_PORT = 6001)
          One complete, self-contained observation per tick. The receiver
          can SYNC and CONCAT ticks: log one Obs per tick and join them, no
          cross-referencing of separate joint / EE streams.

  Priv    (robot -> controller, PUB on tcp://*:PRIV_PORT  = 6003)
          "Privileged" context -- NOT fed to a learned network. The head-
          camera tf (vision/detection consumers) CONCAT the per-tick
          commanded EEDelta block (the expert label for demos, and the
          future action space if we learn in delta space).

  Action  (controller -> robot, SUB on tcp://*:CONTROL_PORT = 6002)
          Two command rails: ControlCmd is EE-space (dual end-effector
          goals + grippers); OverrideCmd is joint-space -- the exact 25
          commanded joint positions in dataset order, the alternative to
          EE control (see below). A later step can send action deltas over
          the same rail once a policy is learned in EEDelta(delta) space.

  Record  (robot -> controller, PUB on tcp://*:RECORD_PORT = 6004)
          A discrete event, not continuous telemetry -- sent only when the
          left Meta Quest controller's A/X or B/Y buttons change state. Its
          own dedicated port keeps this one-shot semantic off the
          fixed-cadence Obs/Priv PUBs.

Payload layouts:

Obs         (gateway_node -> controller, PUB on tcp://*:STATE_PORT)
    layout: 89 doubles = 712 bytes, joint block FIRST (index 0 ==
    arm_l_joint1, mirroring a recorded ffw_il_recorder state.csv row), then
    the EE + grippers block at the tail:
      Joint block [0:75]  -- all N_JOINT_STATE joints in JOINT_STATE_NAMES
      order (the wire contract below):
        [0:25]   joint_pos    (rad / m)
        [25:50]  joint_vel    (rad/s / m/s)
        [50:75]  joint_effort (A)
      EE block [75:89]
        [75:81]  ee0  (x, y, z, rx, ry, rz)      right arm
        [81]     grip0 (0.0 open .. 1.0 close)   right arm gripper
        [82:88]  ee1  (x, y, z, rx, ry, rz)      left arm
        [88]     grip1 (0.0 open .. 1.0 close)   left arm gripper
    ee0/ee1 are the two current end-effector poses (position + RPY
    orientation, see the RPY convention below). grip0/grip1 are the
    normalized gripper openings, matching the spacemouse/quest trigger
    convention: 0.0 = open, 1.0 = close. The joint block is relayed from
    the robot's /joint_states for all N_JOINT_STATE joints; the gateway
    reorders into JOINT_STATE_NAMES order (0.0-fill for any joint missing),
    so the layout is stable regardless of broadcaster ordering. effort is
    the measured motor current (Dynamixel Present Current, in A) -- not a
    controller-computed torque; the base wheel drives run through a
    virtual_dxl interface whose effort may be 0.0 / low-fidelity.

Priv        (gateway_node -> controller, PUB on tcp://*:PRIV_PORT)
    layout: 28 doubles = 224 bytes
      HeadCamTf block [0:16]  row-major 4x4 homogeneous matrix
        [0:4]   row 0 = [m00 m01 m02 m03]
        [4:8]   row 1 = [m10 m11 m12 m13]
        [8:12]  row 2 = [m20 m21 m22 m23]
        [12:16] row 3 = [0   0   0   1   ]
        The head-camera -> control-frame transform:
            p_control = M * [p_camera, 1]
        It is the resolved lookup_transform(control_frame, head_camera_frame)
        from the head_camera_tf_bridge node: R in the upper-left 3x3
        (row-major), translation in column 3 (m03/m13/m23), last row fixed
        to [0 0 0 1].
      EEDelta block [16:28]
        [16:22] d0 = (dx, dy, dz, drx, dry, drz)  right arm commanded delta
        [22:28] d1 = (dx, dy, dz, drx, dry, drz)  left arm commanded delta
        The per-tick delta the spacemouse operator commanded on each arm
        this tick, expressed in the map frame -- the change applied to the
        arm's goal that tick, AFTER the soft-lock / limit-profile clamps.
        dx,dy,dz are in m; drx,dry,drz are extrinsic-XYZ RPY in rad (see
        below). All-zeros means "no command this tick" -- including the
        first tick after a mapper re-base (arm switch / override release).
        An arm not spacemouse-driven right now goes silent and is
        zero-filled on the wire; if neither arm is driven the delta block
        is still sent (zeros), so the frame has a fixed cadence.

ControlCmd  (controller -> robot, SUB on tcp://*:CONTROL_PORT)
    layout: 14 doubles = 112 bytes
      [0:6]   ee0  (x, y, z, rx, ry, rz)      right arm goal
      [6]     grip0 (0.0 open .. 1.0 close)   right arm gripper
      [7:13]  ee1  (x, y, z, rx, ry, rz)      left arm goal
      [13]    grip1 (0.0 open .. 1.0 close)   left arm gripper
    ee0/ee1 are the two commanded end-effector poses (position + RPY
    orientation). grip0/grip1 command the grippers, forwarded to the quest
    trigger channel (TRACK state only, like the physical spacemouse trigger).
    0.0 = open, 1.0 = close.

OverrideCmd (controller -> robot, SUB on tcp://*:CONTROL_PORT)
    layout: 25 doubles = 200 bytes, all in JOINT_STATE_NAMES order -- the
    same fixed order as the Obs joint block above / a dataset q-column row
      [0:25]  joint_pos   commanded joint positions (rad / m)
          index N == Obs joint index N == dataset q-column index N
    Joint-space command rail: the controller's full commanded joint vector.
    It is the alternative to the EE-space ControlCmd rail -- the controller
    commands either EE goals (ControlCmd) or the exact 25 joint positions in
    this order (OverrideCmd). Not a latch: there is no 0/1 engage bit; the
    command stream itself is the control signal, and the receiver holds the
    newest frame and treats staleness like ControlCmd.

Record      (gateway_node -> controller, PUB on tcp://*:RECORD_PORT)
    layout: 1 int32 = 4 bytes
      [0]  value  bit-encoded from the left controller's A/X and B/Y
                  buttons: 0 = none, 1 = A/X, 2 = B/Y, 3 = both
    Sent only on change (not every tick like Obs/Priv), from
    ffw_spacemouse_msgs/LeftControlOverride.record (decoded by
    quest_to_ros2.py, relayed by gateway_node).

RPY convention: extrinsic XYZ / intrinsic ZYX, i.e. the rotation matrix is
R = Rx(roll) * Ry(pitch) * Rz(yaw), matching the spacemouse teleop stack
(joy_hand rpy_to_matrix). rpy_to_quat / quat_to_rpy use this convention.

Both ends must agree on this layout. If the controller side (e.g. SERL
hil-serl) ships its own protocol.py, drop it in over this one -- the
encoders/decoders are the only thing that has to match across the link.
"""

import math
import struct
import time

# --- Message type identifiers (header field 0) --------------------------
MSG_OBS = 1          # gateway_node -> controller: joint block + EE block
MSG_CONTROL_CMD = 2  # controller -> robot: two 6-DOF EE targets
MSG_PRIV = 3         # gateway_node -> controller: head-cam tf + EEDelta
MSG_OVERRIDE = 4     # controller -> robot: joint-space command rail (25 qpos)
MSG_RECORD = 5       # gateway_node -> controller: left A/X + B/Y button event

# --- Wire contract: relayed /joint_states order (Obs joint block) --------
# Fixed joint order for the relayed /joint_states. The gateway reorders
# incoming /joint_states into this order (0.0-fill for any joint missing from
# the message), so the client addresses joints by index from this table.
# Model-specific to ffw_sg2_smtm: 22 follower joints + 3 base wheel drives.
#
# The order MATCHES the ffw_il_recorder dataset q-column order (state.csv):
# arm_l 1-7, arm_r 1-7, grippers, head, then the wheels/lift block. Obs joint
# index N therefore lines up 1:1 with dataset q-column index N.
JOINT_STATE_NAMES = (
    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
    "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "arm_r_joint1",
    "arm_r_joint2", "arm_r_joint3", "arm_r_joint4", "arm_r_joint5",
    "arm_r_joint6", "arm_r_joint7", "gripper_l_joint1", "gripper_r_joint1",
    "head_joint1", "head_joint2",
    "left_wheel_drive", "left_wheel_steer", "lift_joint",
    "rear_wheel_drive", "rear_wheel_steer",
    "right_wheel_drive", "right_wheel_steer",
)
N_JOINT_STATE = len(JOINT_STATE_NAMES)  # 25

# Convenience indices into JOINT_STATE_NAMES for the two gripper joints.
GRIP_R_JOINT_IDX = JOINT_STATE_NAMES.index("gripper_r_joint1")  # 15
GRIP_L_JOINT_IDX = JOINT_STATE_NAMES.index("gripper_l_joint1")  # 14

# Header: type (int32) + timestamp (float64) = 12 bytes, little-endian.
_HEADER = struct.Struct("<id")

# Payload structs (per type).
# _CTRL_STRUCT is the ControlCmd payload AND the Obs EE tail block -- both are
# two 6-DOF poses + two gripper values, flattened by _ctrl_values. Obs packs
# that 14-double EE block AFTER its 75-double joint block (see docstring), so
# it is no longer a byte-identical prefix of the frame. The decoder still
# dispatches on the header type id.
_CTRL_STRUCT = struct.Struct("<14d")                        # 112 bytes
_OBS_STRUCT = struct.Struct("<" + "d" * (3 * N_JOINT_STATE + 14))  # 712 bytes
_PRIV_STRUCT = struct.Struct("<" + "d" * (16 + 12))         # 224 bytes
_OVERRIDE_STRUCT = struct.Struct("<" + "d" * N_JOINT_STATE)   # 200 bytes
_RECORD_STRUCT = struct.Struct("<i")                         # 4 bytes

# Expected frame size per type (header + payload), for length validation.
_FRAME_SIZES = {
    MSG_OBS: _HEADER.size + _OBS_STRUCT.size,              # 724 B
    MSG_CONTROL_CMD: _HEADER.size + _CTRL_STRUCT.size,     # 124 B
    MSG_PRIV: _HEADER.size + _PRIV_STRUCT.size,            # 236 B
    MSG_OVERRIDE: _HEADER.size + _OVERRIDE_STRUCT.size,    # 212 B
    MSG_RECORD: _HEADER.size + _RECORD_STRUCT.size,        # 16 B
}


def _frame(msg_type, payload, ts):
    """Prepend the header to an already-packed payload."""
    if ts is None:
        ts = time.time()
    return _HEADER.pack(msg_type, ts) + payload


def clamp01(v):
    """Clamp a gripper value to [0, 1] (0.0 = open .. 1.0 = close)."""
    return max(0.0, min(1.0, float(v)))


def _unframe(data, msg_type, name):
    """Strip and validate the header. Returns (payload_bytes, ts)."""
    expected = _FRAME_SIZES[msg_type]
    if len(data) != expected:
        raise ValueError(f"{name} expects {expected} bytes, got {len(data)}")
    got_type, ts = _HEADER.unpack_from(data, 0)
    if got_type != msg_type:
        raise ValueError(
            f"{name}: type {got_type}, expected {msg_type}")
    return data[_HEADER.size:], ts


class Obs:
    """One complete observation: dual EE poses + grippers + joint state.

    ee = (ee0, ee1), each a 6-tuple pose (x, y, z, rx, ry, rz);
    gripper = (grip0, grip1), each in [0, 1] -- 0.0 = open, 1.0 = close;
    joint_pos / joint_vel / joint_effort are lists of length N_JOINT_STATE
    in JOINT_STATE_NAMES order.

    Keepers of the old drain_ee surface read .ee / .gripper and ignore the
    joint block; nothing else changes for them.
    """

    __slots__ = ("ee", "gripper", "joint_pos", "joint_vel", "joint_effort")

    def __init__(self, ee=((0.0,) * 6, (0.0,) * 6), gripper=(0.0, 0.0)):
        self.ee = ee
        self.gripper = gripper
        self.joint_pos = [0.0] * N_JOINT_STATE
        self.joint_vel = [0.0] * N_JOINT_STATE
        self.joint_effort = [0.0] * N_JOINT_STATE


class Priv:
    """Privileged context: head-cam tf + per-tick commanded deltas.

    matrix is a 16-tuple, row-major homogeneous matrix (see the module
    docstring); default is the identity. delta = (d0, d1), each a 6-tuple
    (dx, dy, dz, drx, dry, drz) in the map frame; all-zeros means "no
    command this tick".
    """

    __slots__ = ("matrix", "delta")

    _IDENTITY = (1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0)

    def __init__(self, matrix=None, delta=((0.0,) * 6, (0.0,) * 6)):
        self.matrix = tuple(matrix) if matrix is not None else self._IDENTITY
        self.delta = delta


class ControlCmd:
    """Dual end-effector targets + grippers.

    ee = (ee0, ee1), each a 6-tuple pose (x, y, z, rx, ry, rz);
    gripper = (grip0, grip1), each in [0, 1] -- 0.0 = open, 1.0 = close,
    matching the quest trigger. Default gripper is (0.0, 0.0) = open, so a
    pose-only controller behaves safely.
    """

    __slots__ = ("ee", "gripper")

    def __init__(self, ee=((0.0,) * 6, (0.0,) * 6), gripper=(0.0, 0.0)):
        self.ee = ee
        self.gripper = gripper


class OverrideCmd:
    """Joint-space command: the exact 25 commanded joint positions.

    joint_pos is a list of length N_JOINT_STATE in JOINT_STATE_NAMES order
    (index N == Obs joint index N == dataset q-column N). This is the
    joint-space alternative to the EE-space ControlCmd rail -- not a latch,
    and there is no 0/1 value: the command stream itself is the control
    signal. Defaults to all zeros (a commanded all-joints-home pose).
    """

    __slots__ = ("joint_pos",)

    def __init__(self, joint_pos=None):
        self.joint_pos = (
            list(joint_pos) if joint_pos is not None else [0.0] * N_JOINT_STATE
        )


class Record:
    """A discrete left-controller button event, bit-encoded.

    value: 0 = none, 1 = A/X, 2 = B/Y, 3 = both. Sent only on change, unlike
    the fixed-cadence Obs/Priv streams.
    """

    __slots__ = ("value",)

    def __init__(self, value=0):
        self.value = int(value)


def _ctrl_values(ee, gripper):
    """Flatten ee + gripper into the 14-value wire order (ee0 grip0 ee1 grip1)."""
    return ee[0] + (gripper[0],) + ee[1] + (gripper[1],)


def _delta_values(delta):
    """Flatten delta into the 12-value wire order (d0 + d1)."""
    return delta[0] + delta[1]


def encode_obs(obs: Obs, ts=None) -> bytes:
    """Encode Obs, auto-stamping the send time if ts is None."""
    vals = list(obs.joint_pos + obs.joint_vel + obs.joint_effort)
    vals += _ctrl_values(obs.ee, obs.gripper)
    payload = _OBS_STRUCT.pack(*vals)
    return _frame(MSG_OBS, payload, ts)


def decode_obs(data: bytes):
    """Decode Obs. Returns (obs, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_OBS, "Obs")
    vals = _OBS_STRUCT.unpack(payload)
    n = N_JOINT_STATE
    ee0 = 3 * n  # joint block (pos + vel + effort) leads the frame; EE at tail
    obs = Obs((vals[ee0:ee0 + 6], vals[ee0 + 7:ee0 + 13]),
              gripper=(vals[ee0 + 6], vals[ee0 + 13]))
    obs.joint_pos = list(vals[0:n])
    obs.joint_vel = list(vals[n:2 * n])
    obs.joint_effort = list(vals[2 * n:3 * n])
    return obs, ts


def encode_priv(priv: Priv, ts=None) -> bytes:
    """Encode Priv, auto-stamping the send time if ts is None."""
    payload = _PRIV_STRUCT.pack(*(priv.matrix + _delta_values(priv.delta)))
    return _frame(MSG_PRIV, payload, ts)


def decode_priv(data: bytes):
    """Decode Priv. Returns (priv, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_PRIV, "Priv")
    vals = _PRIV_STRUCT.unpack(payload)
    return Priv(matrix=vals[0:16], delta=(vals[16:22], vals[22:28])), ts


def encode_control(cmd: ControlCmd, ts=None) -> bytes:
    """Encode ControlCmd, auto-stamping the send time if ts is None."""
    payload = _CTRL_STRUCT.pack(*_ctrl_values(cmd.ee, cmd.gripper))
    return _frame(MSG_CONTROL_CMD, payload, ts)


def decode_control(data: bytes):
    """Decode ControlCmd. Returns (cmd, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_CONTROL_CMD, "ControlCmd")
    vals = _CTRL_STRUCT.unpack(payload)
    return ControlCmd((vals[0:6], vals[7:13]), gripper=(vals[6], vals[13])), ts


def encode_override(cmd: OverrideCmd, ts=None) -> bytes:
    """Encode OverrideCmd, auto-stamping the send time if ts is None."""
    if len(cmd.joint_pos) != N_JOINT_STATE:
        raise ValueError(
            f"OverrideCmd.joint_pos must be exactly {N_JOINT_STATE} values "
            f"in JOINT_STATE_NAMES order, got {len(cmd.joint_pos)}")
    payload = _OVERRIDE_STRUCT.pack(*cmd.joint_pos)
    return _frame(MSG_OVERRIDE, payload, ts)


def decode_override(data: bytes):
    """Decode OverrideCmd. Returns (cmd, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_OVERRIDE, "OverrideCmd")
    return OverrideCmd(list(_OVERRIDE_STRUCT.unpack(payload))), ts


def encode_record(rec: Record, ts=None) -> bytes:
    """Encode Record, auto-stamping the send time if ts is None."""
    payload = _RECORD_STRUCT.pack(rec.value)
    return _frame(MSG_RECORD, payload, ts)


def decode_record(data: bytes):
    """Decode Record. Returns (rec, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_RECORD, "Record")
    (value,) = _RECORD_STRUCT.unpack(payload)
    return Record(value), ts


def rpy_to_quat(rx, ry, rz):
    """Extrinsic-XYZ RPY (joy_hand convention) to unit quaternion (w, x, y, z).

    Derived from q = qx(rx) * qy(ry) * qz(rz), matching the matrix
    R = Rx(rx) * Ry(ry) * Rz(rz).
    """
    cr, sr = math.cos(rx / 2.0), math.sin(rx / 2.0)
    cp, sp = math.cos(ry / 2.0), math.sin(ry / 2.0)
    cy, sy = math.cos(rz / 2.0), math.sin(rz / 2.0)
    w = cr * cp * cy - sr * sp * sy
    x = sr * cp * cy + cr * sp * sy
    y = cr * sp * cy - sr * cp * sy
    z = cr * cp * sy + sr * sp * cy
    return (w, x, y, z)


def quat_to_rpy(w, x, y, z):
    """Unit quaternion (w, x, y, z) to extrinsic-XYZ RPY (rx, ry, rz)."""
    roll = math.atan2(2.0 * (w * x - y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y + x * z))))
    yaw = math.atan2(2.0 * (w * z - x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)
