"""Wire messages for the peg-hole server.

Standalone: a client needs only this file (struct + numpy). On the server, Obs
is encoded with the robot gateway's own protocol module
(ffw_zmqinterface/protocol.py) when it is importable, so the sim publishes
byte-identical Obs; decode_obs() below decodes that frame without it.

Every frame starts with the gateway's 12-byte header: int32 type, float64
timestamp (sender clock).

Ports, offsets from a base (6001 on the robot):
  OBS +0 (PUB), CONTROL +1 (SUB on the server), STATUS +4 (PUB), IMAGES +5 (PUB).
  +2 (Priv) and +3 (Record) belong to the robot gateway and are not used here.

Obs (type 1), 131 float64 after the header (1060-byte frame):
  [0:25]    joint_pos     JOINT_STATE_NAMES order (rad / m)
  [25:50]   joint_vel
  [50:75]   joint_effort  (A)
  [75:81]   ee0 = right EE (x, y, z, rx, ry, rz), base_link, R = Rx Ry Rz
  [81]      grip0 = right gripper, 0 open .. 1 closed
  [82:88]   ee1 = left EE
  [88]      grip1 = left gripper
  [89:95]   base_marker   base_link pose in the AprilTag marker frame
  [95:101]  ee0_marker    right EE pose in the marker frame
  [101:113] diff0         right EE margins to the active limit box (x_lo, x_hi,
                          y_lo, ..., yaw_hi); NO_LIMIT_DIFF (10.0) = no box
  [113:119] ee1_marker
  [119:131] diff1
  The marker block stays at its defaults (zeros / NO_LIMIT_DIFF) in the simulator.

New messages:
  ControlCmdDelta (6)  controller -> server, CONTROL. 14 float64: right EE
                       (dx, dy, dz, droll, dpitch, dyaw, grip) then left EE
                       (same). Translation in base_link; rotation applied in
                       base_link as R_new = R_delta @ R, R_delta from intrinsic
                       X-Y-Z angles (= the Obs rpy convention R = Rx Ry Rz).
                       grip is carried but not acted on (the peg stays gripped).
  EnvCmd (8)           controller -> server, CONTROL. cmd (uint8), seed (int64),
                       episode_id (uint32), 8 float64 params (reserved).
  EnvStatus (9)        server -> controller, STATUS. state, backend, reset_ok,
                       success, terminated, truncated (uint8 each); episode_id,
                       step, deltas_received (uint32 each); reward, depth,
                       lateral, contact_force (float64 each).
  Images (10)          server -> controller, IMAGES, multipart:
                       [header + episode_id, step (uint32), height, width
                       (uint16), right RGB bytes, left RGB bytes]. Sent right
                       after the Obs of the same tick; pair by (episode_id, step).

MSG_CONTROL_DELTA = 6 follows the HIL-SERL spec. Agreed 2026-10-02: it keeps 6,
and the robot gateway's SetMode (also 6 today) moves to another number.
"""
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ffw_zmqinterface"))
try:
    from ffw_zmqinterface import protocol as gw   # robot workstation: encode Obs exactly as the gateway
except ImportError:
    gw = None                                     # client side: everything below is self-contained

# Port offsets from the base port.
PORT_OBS, PORT_CONTROL, PORT_STATUS, PORT_IMAGES = 0, 1, 4, 5

MSG_OBS = 1
MSG_CONTROL_DELTA = 6
MSG_ENV_CMD = 8
MSG_ENV_STATUS = 9
MSG_IMAGES = 10

# EnvCmd.cmd
RESET, PAUSE, RESUME, ABORT, PING = 1, 2, 3, 4, 5
CMD_NAMES = {RESET: "RESET", PAUSE: "PAUSE", RESUME: "RESUME", ABORT: "ABORT", PING: "PING"}
# EnvStatus.state
IDLE, RESETTING, READY, RUNNING, PAUSED, DONE, FAULT = range(7)
STATE_NAMES = ["IDLE", "RESETTING", "READY", "RUNNING", "PAUSED", "DONE", "FAULT"]
# EnvStatus.backend
BACKEND_SIM, BACKEND_REAL = 0, 1

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
N_JOINT_STATE = len(JOINT_STATE_NAMES)            # 25
OBS_N_DOUBLES = 131
NO_LIMIT_DIFF = 10.0

_HEADER = struct.Struct("<id")                    # int32 type, float64 timestamp
_OBS = struct.Struct("<" + "d" * OBS_N_DOUBLES)    # 1048 bytes -> 1060-byte frame
_DELTA = struct.Struct("<14d")
_ENV_CMD = struct.Struct("<BqI8d")
_ENV_STATUS = struct.Struct("<6B3I4d")
_IMAGES = struct.Struct("<2I2H")

if gw is not None:                                # the copy above must match the gateway exactly
    assert gw._HEADER.format == _HEADER.format
    assert tuple(gw.JOINT_STATE_NAMES) == JOINT_STATE_NAMES
    assert gw._FRAME_SIZES[gw.MSG_OBS] == _HEADER.size + _OBS.size


def msg_type(data):
    return _HEADER.unpack_from(data, 0)[0]


def _frame(t, payload, ts):
    return _HEADER.pack(t, time.time() if ts is None else ts) + payload


def _payload(data, t, st):
    if len(data) != _HEADER.size + st.size:
        raise ValueError(f"type {t}: expected {_HEADER.size + st.size} bytes, got {len(data)}")
    got, ts = _HEADER.unpack_from(data, 0)
    if got != t:
        raise ValueError(f"type {got}, expected {t}")
    return st.unpack_from(data, _HEADER.size), ts


def encode_delta(right7, left7, ts=None):
    return _frame(MSG_CONTROL_DELTA, _DELTA.pack(*right7, *left7), ts)


def decode_delta(data):
    v, ts = _payload(data, MSG_CONTROL_DELTA, _DELTA)
    return tuple(v[:7]), tuple(v[7:]), ts


def encode_env_cmd(cmd, seed=0, episode_id=0, params=(0.0,) * 8, ts=None):
    return _frame(MSG_ENV_CMD, _ENV_CMD.pack(cmd, seed, episode_id, *params), ts)


def decode_env_cmd(data):
    v, ts = _payload(data, MSG_ENV_CMD, _ENV_CMD)
    return {"cmd": v[0], "seed": v[1], "episode_id": v[2], "params": v[3:]}, ts


def encode_status(s, ts=None):
    return _frame(MSG_ENV_STATUS, _ENV_STATUS.pack(
        s["state"], s["backend"], s["reset_ok"], s["success"], s["terminated"], s["truncated"],
        s["episode_id"], s["step"], s["deltas_received"],
        s["reward"], s["depth"], s["lateral"], s["contact_force"]), ts)


def decode_status(data):
    v, ts = _payload(data, MSG_ENV_STATUS, _ENV_STATUS)
    keys = ("state", "backend", "reset_ok", "success", "terminated", "truncated",
            "episode_id", "step", "deltas_received", "reward", "depth", "lateral", "contact_force")
    return dict(zip(keys, v)), ts


def encode_images(episode_id, step, right, left, ts=None):
    """Multipart frames for the two wrist images (HxWx3 uint8, same size)."""
    h, w = right.shape[:2]
    head = _frame(MSG_IMAGES, _IMAGES.pack(episode_id, step, h, w), ts)
    return [head, right.tobytes(), left.tobytes()]


def decode_images(frames):
    import numpy as np
    (episode_id, step, h, w), ts = _payload(frames[0], MSG_IMAGES, _IMAGES)
    imgs = [np.frombuffer(f, dtype=np.uint8).reshape(h, w, 3) for f in frames[1:3]]
    return {"episode_id": episode_id, "step": step, "right": imgs[0], "left": imgs[1]}, ts


def decode_obs(data):
    """Decode a gateway Obs frame (1060 bytes) into named numpy arrays."""
    import numpy as np
    v, ts = _payload(data, MSG_OBS, _OBS)
    v = np.asarray(v)
    n = N_JOINT_STATE
    return {
        "joint_pos": v[0:n], "joint_vel": v[n:2 * n], "joint_effort": v[2 * n:3 * n],
        "ee_right": v[75:81], "grip_right": float(v[81]),
        "ee_left": v[82:88], "grip_left": float(v[88]),
        "base_marker": v[89:95],
        "ee_right_marker": v[95:101], "limit_diff_right": v[101:113],
        "ee_left_marker": v[113:119], "limit_diff_left": v[119:131],
    }, ts
