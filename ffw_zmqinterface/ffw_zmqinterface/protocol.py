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

Payload layouts (unchanged from before the header):

RobotState  (robot_node -> controller, PUB on tcp://*:STATE_PORT)
    layout: 60 doubles = 480 bytes
      [0:20]   joint_pos   (rad / m)
      [20:40]  joint_vel   (rad/s / m/s)
      [40:60]  joint_acc   (rad/s^2 / m/s^2)

EEState     (gateway_node -> controller, PUB on tcp://*:STATE_PORT)
    layout: 12 doubles = 96 bytes
      [0:6]   ee0  (x, y, z, rx, ry, rz)
      [6:12]  ee1  (x, y, z, rx, ry, rz)
    ee0/ee1 are the two current end-effector poses (position + RPY
    orientation), right arm and left arm.

ControlCmd  (controller -> robot, SUB on tcp://*:CONTROL_PORT)
    layout: 12 doubles = 96 bytes
      [0:6]   ee0  (x, y, z, rx, ry, rz)
      [6:12]  ee1  (x, y, z, rx, ry, rz)
    ee0/ee1 are the two commanded end-effector poses (position + RPY
    orientation), right arm and left arm.

HeadCamTf   (gateway_node -> controller, PUB on tcp://*:STATE_PORT)
    layout: 16 doubles = 128 bytes, 4x4 homogeneous matrix, ROW-MAJOR
      [0:4]   row 0 = [m00 m01 m02 m03]
      [4:8]   row 1 = [m10 m11 m12 m13]
      [8:12]  row 2 = [m20 m21 m22 m23]
      [12:16] row 3 = [0   0   0   1   ]
    The head-camera -> control-frame transform: maps a detection expressed in
    camera coordinates into the frame the robot is controlled in (the control
    frame, e.g. 'base_link'):
        p_control = M * [p_camera, 1]
    It is the resolved lookup_transform(control_frame, head_camera_frame)
    from the head_camera_tf_bridge node: R in the upper-left 3x3 (row-major),
    translation in column 3 (m03/m13/m23), last row fixed to [0 0 0 1].

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

N_JOINTS = 20

# --- Message type identifiers (header field 0) --------------------------
MSG_ROBOT_STATE = 0   # robot_node -> controller: 20 joints pos/vel/acc
MSG_EE_STATE = 1      # gateway_node -> controller: two 6-DOF EE poses
MSG_CONTROL_CMD = 2   # controller -> robot: two 6-DOF EE targets
MSG_HEAD_CAM_TF = 3   # gateway_node -> controller: camera->control 4x4

# Header: type (int32) + timestamp (float64) = 12 bytes, little-endian.
_HEADER = struct.Struct("<id")

# Payload structs (per type).
_STATE_STRUCT = struct.Struct("<" + "d" * (3 * N_JOINTS))  # 480 bytes
_CTRL_STRUCT = struct.Struct("<12d")                        # 96 bytes
_TF_STRUCT = struct.Struct("<16d")                          # 128 bytes

# Expected frame size per type (header + payload), for length validation.
_FRAME_SIZES = {
    MSG_ROBOT_STATE: _HEADER.size + _STATE_STRUCT.size,
    MSG_EE_STATE: _HEADER.size + _CTRL_STRUCT.size,
    MSG_CONTROL_CMD: _HEADER.size + _CTRL_STRUCT.size,
    MSG_HEAD_CAM_TF: _HEADER.size + _TF_STRUCT.size,
}


def _frame(msg_type, payload, ts):
    """Prepend the header to an already-packed payload."""
    if ts is None:
        ts = time.time()
    return _HEADER.pack(msg_type, ts) + payload


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


class RobotState:
    """Joint pos/vel/acc for N_JOINTS joints."""

    __slots__ = ("joint_pos", "joint_vel", "joint_acc")

    def __init__(self):
        self.joint_pos = [0.0] * N_JOINTS
        self.joint_vel = [0.0] * N_JOINTS
        self.joint_acc = [0.0] * N_JOINTS


class ControlCmd:
    """Dual end-effector targets: ee = (ee0, ee1), each a 6-tuple."""

    __slots__ = ("ee",)

    def __init__(self, ee=((0.0,) * 6, (0.0,) * 6)):
        self.ee = ee


class EEState:
    """Dual current end-effector poses: ee = (ee0, ee1), each a 6-tuple."""

    __slots__ = ("ee",)

    def __init__(self, ee=((0.0,) * 6, (0.0,) * 6)):
        self.ee = ee


class HeadCamTf:
    """Head-camera -> control-frame transform as a row-major 4x4.

    `matrix` is a 16-tuple, row-major homogeneous matrix:
        row 0 = (m00 m01 m02 m03)   row 1 = (m10 m11 m12 m13)
        row 2 = (m20 m21 m22 m23)   row 3 = (0   0   0   1   )
    So p_control = M * [p_camera, 1]. Default is the identity.
    """

    __slots__ = ("matrix",)

    _IDENTITY = (1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0)

    def __init__(self, matrix=None):
        self.matrix = tuple(matrix) if matrix is not None else self._IDENTITY


def encode_robot_state(state: RobotState, ts=None) -> bytes:
    """Encode RobotState, auto-stamping the send time if ts is None."""
    payload = _STATE_STRUCT.pack(*(state.joint_pos + state.joint_vel +
                                   state.joint_acc))
    return _frame(MSG_ROBOT_STATE, payload, ts)


def decode_robot_state(data: bytes):
    """Decode RobotState. Returns (state, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_ROBOT_STATE, "RobotState")
    vals = _STATE_STRUCT.unpack(payload)
    n = N_JOINTS
    st = RobotState()
    st.joint_pos = list(vals[0:n])
    st.joint_vel = list(vals[n:2 * n])
    st.joint_acc = list(vals[2 * n:3 * n])
    return st, ts


def encode_control(cmd: ControlCmd, ts=None) -> bytes:
    """Encode ControlCmd, auto-stamping the send time if ts is None."""
    payload = _CTRL_STRUCT.pack(*(cmd.ee[0] + cmd.ee[1]))
    return _frame(MSG_CONTROL_CMD, payload, ts)


def decode_control(data: bytes):
    """Decode ControlCmd. Returns (cmd, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_CONTROL_CMD, "ControlCmd")
    vals = _CTRL_STRUCT.unpack(payload)
    return ControlCmd((vals[0:6], vals[6:12])), ts


def encode_ee_state(state: EEState, ts=None) -> bytes:
    """Encode EEState, auto-stamping the send time if ts is None."""
    payload = _CTRL_STRUCT.pack(*(state.ee[0] + state.ee[1]))
    return _frame(MSG_EE_STATE, payload, ts)


def decode_ee_state(data: bytes):
    """Decode EEState. Returns (state, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_EE_STATE, "EEState")
    vals = _CTRL_STRUCT.unpack(payload)
    return EEState((vals[0:6], vals[6:12])), ts


def encode_head_cam_tf(tf: HeadCamTf, ts=None) -> bytes:
    """Encode HeadCamTf, auto-stamping the send time if ts is None."""
    payload = _TF_STRUCT.pack(*tf.matrix)
    return _frame(MSG_HEAD_CAM_TF, payload, ts)


def decode_head_cam_tf(data: bytes):
    """Decode HeadCamTf. Returns (tf, ts) where ts is the sender's."""
    payload, ts = _unframe(data, MSG_HEAD_CAM_TF, "HeadCamTf")
    return HeadCamTf(_TF_STRUCT.unpack(payload)), ts


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
