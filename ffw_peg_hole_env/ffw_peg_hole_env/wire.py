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

  Frame (12)           server -> controller, OBS, real robot: one per 15 Hz tick.
                       float64 obs[OBS_N], float64 priv[PRIV_N], then the tag:
                       episode_id, step (uint32); frame_state, reason (uint8);
                       reward (float64); action (6 float64: the right-arm delta
                       actually applied this tick -- the policy's, or the
                       machine's on INTERVENTION frames). FRAME_BYTES long.
                       obs = what a deployed actor can measure; priv = everything
                       the server derives (critic / logging). Field tables:
                       OBS_FIELDS, PRIV_FIELDS below (name, size, meaning);
                       decode_frame() returns them as named arrays, FRAME.md
                       documents them. NaN = not available in this state.
                       frame_state: FS_IDLE, FS_POLICY, FS_INTERVENTION,
                       FS_TERMINATED, FS_RESET, FS_FAULT. reason (on TERMINATED):
                       TR_SUCCESS, TR_JAM, TR_RIM, TR_BLOCKED, TR_TIMEOUT,
                       TR_SAFETY, TR_INTERVENTIONS, TR_ABORT (TR_OFF_AXIS = 9 is
                       reserved, no longer sent). Store POLICY and INTERVENTION
                       frames, TERMINATED ends the episode, skip RESET / IDLE / FAULT.
  (type 11 was the first Frame: the gateway's 131-double Obs + the tag; retired
  2026-10-04, its Obs now travels inside priv as "gateway_obs".)

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
MSG_FRAME = 12                                    # 11 = the retired Obs + tag Frame

# EnvCmd.cmd
RESET, PAUSE, RESUME, ABORT, PING = 1, 2, 3, 4, 5
CMD_NAMES = {RESET: "RESET", PAUSE: "PAUSE", RESUME: "RESUME", ABORT: "ABORT", PING: "PING"}
# EnvStatus.state
IDLE, RESETTING, READY, RUNNING, PAUSED, DONE, FAULT = range(7)
STATE_NAMES = ["IDLE", "RESETTING", "READY", "RUNNING", "PAUSED", "DONE", "FAULT"]
# Frame.frame_state / Frame.reason
FS_IDLE, FS_POLICY, FS_INTERVENTION, FS_TERMINATED, FS_RESET, FS_FAULT = range(6)
FS_NAMES = ["IDLE", "POLICY", "INTERVENTION", "TERMINATED", "RESET", "FAULT"]
TR_NONE, TR_SUCCESS, TR_JAM, TR_RIM, TR_BLOCKED, TR_TIMEOUT, TR_SAFETY, TR_INTERVENTIONS, TR_ABORT, TR_OFF_AXIS = range(10)
TR_NAMES = ["NONE", "SUCCESS", "JAM", "RIM", "BLOCKED", "TIMEOUT", "SAFETY", "INTERVENTIONS", "ABORT", "OFF_AXIS"]
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
_TAG = struct.Struct("<2I2Bd6d")                  # episode_id, step, frame_state, reason, reward, action

# --- Frame layout (name, size, meaning). Joint order JOINT_STATE_NAMES; poses base_link,
# (x, y, z, roll, pitch, yaw) with R = Rx Ry Rz; m, rad, s, N, A unless stated.
OBS_FIELDS = (
    ("joints", 3 * N_JOINT_STATE, "per joint [pos, vel, effort], interleaved: joint j at 3j..3j+2; "
                                  "effort in A on the arm joints, driver units elsewhere"),
    ("ee_right", 6, "right IK EE pose, achieved (/ik_solver/achieved_ee_pose_r), base_link"),
    ("ee_right_vel", 6, "right EE velocity (vx, vy, vz, wx, wy, wz), base_link, finite difference"),
    ("ee_left", 6, "left IK EE pose, achieved, base_link"),
    ("ee_left_vel", 6, "left EE velocity"),
    ("limit_right", 12, "right EE margins to the peg_hole_safe_real_r box in peg_hole_frame_r: "
                        "x_lo, x_hi, y_lo, y_hi, z_lo, z_hi, roll_lo, ..., yaw_hi; lo = v - min, "
                        "hi = max - v (< 0 = past the bound); NO_LIMIT_DIFF = no box / frame"),
    ("limit_left", 12, "left EE margins, peg_hole_safe_real_l in peg_hole_frame_l"),
)
PRIV_FIELDS = (
    # reward
    ("reward", 1, "this tick's reward (= tag.reward)"),
    ("return", 1, "episode return so far, this tick included"),
    ("r_success", 1, "reward term: success bonus (terminal)"),
    ("r_shape", 1, "reward term: potential-based progress"),
    ("r_force", 1, "reward term: push-back penalty"),
    ("r_fail", 1, "reward term: failure penalty (terminal)"),
    # force estimator (right arm currents -> J^T solve at right_peg_site, driving K)
    ("push_back", 1, "F_pb: push-back on the peg along its axis, N (> 0 = pushed back) = ref - axial"),
    ("push_back_peak", 1, "highest F_pb since the peg could first touch the hole, N"),
    ("force_axial", 1, "raw axial force estimate before the reference, N"),
    ("force_ref", 1, "free-motion reference (median while the peg cannot touch), N"),
    # task geometry (calibrated hole axis = modelled axis + the eye-calibrated peg offset)
    ("depth", 1, "peg tip below the rim, m (< 0 = above), measured"),
    ("depth_cmd", 1, "the same for the commanded pose, m"),
    ("lateral", 1, "peg tip distance from the calibrated hole axis, m"),
    ("lateral_yz", 2, "the same as (y, z) components in the hole tool frame, m"),
    ("tilt", 1, "angle between peg and hole axes, deg"),
    ("hole_pose", 6, "hole tool point pose (x = insertion axis, up), base_link"),
    ("peg_in_hole", 6, "peg tool pose in the hole tool frame (measured)"),
    ("ee_right_cmd", 6, "right commanded IK EE pose after the server's clamps, base_link"),
    ("hole_offset", 2, "calibrated peg-tool (y, z) offset in use, m"),
    # control
    ("policy_delta", 6, "the policy's raw delta received for this tick (NaN = none / stale)"),
    ("delta_age", 1, "s since the newest policy delta arrived (NaN = none yet)"),
    ("clamped", 1, "1 = the server's box / above-rim clamp changed the command this tick"),
    # env flags / episode state
    ("mode", 1, "machine mode driving the NEXT tick: 0 idle, 1 policy, 2 pull_out, 3 trace_back, "
                "4 reset, 5 fault"),
    ("interventions", 1, "machine interventions so far this episode"),
    ("t_episode", 1, "s since the episode started"),
    ("can_touch", 1, "1 = the peg can touch the hole (tip within 1 mm of the rim or lower)"),
    ("in_zone", 1, "1 = within the 2 cm interaction zone of the calibrated axis"),
    ("success", 1, "1 = success condition met this tick"),
    ("failed", 1, "1 = reward-rule failure (jam / rim) this tick"),
    ("rim_strike", 1, "1 = the failure was a rim strike"),
    ("blocked", 1, "1 = blocked progress (command descends, peg does not)"),
    ("safety", 1, "1 = live safety stop this tick (gear guard, hard edge, left j7, hole pushed)"),
    ("hole_shift", 1, "hole moved sideways since the peg was last clear, m"),
    ("j7_left_change", 1, "|left j7 current change| since the peg was last clear, mA (gear guard input)"),
    ("limit_frame_ok", 2, "1 = peg_hole_frame_r / _l found on TF (obs limit_* valid)"),
    ("image_t", 2, "receive time (time.time) of the right / left wrist image paired with this tick; NaN = none"),
    # everything else: the robot gateway's full Obs, as it would publish it
    ("gateway_obs", OBS_N_DOUBLES, "the gateway's 131-double Obs (decode_gateway_obs() names it): "
                                   "joint blocks, ee + grippers, marker block"),
)
OBS_N = sum(n for _, n, _ in OBS_FIELDS)          # 123
PRIV_N = sum(n for _, n, _ in PRIV_FIELDS)
_OBS_V = struct.Struct("<%dd" % OBS_N)
_PRIV_V = struct.Struct("<%dd" % PRIV_N)
FRAME_BYTES = _HEADER.size + _OBS_V.size + _PRIV_V.size + _TAG.size
MODE_CODES = {"idle": 0, "policy": 1, "pull_out": 2, "trace_back": 3, "reset": 4, "fault": 5}


def _offsets(fields):
    out, i = {}, 0
    for name, n, _ in fields:
        out[name] = (i, i + n)
        i += n
    return out


OBS_SLICES, PRIV_SLICES = _offsets(OBS_FIELDS), _offsets(PRIV_FIELDS)

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


def _gateway_obs_dict(v):
    import numpy as np
    v = np.asarray(v)
    n = N_JOINT_STATE
    return {
        "joint_pos": v[0:n], "joint_vel": v[n:2 * n], "joint_effort": v[2 * n:3 * n],
        "ee_right": v[75:81], "grip_right": float(v[81]),
        "ee_left": v[82:88], "grip_left": float(v[88]),
        "base_marker": v[89:95],
        "ee_right_marker": v[95:101], "limit_diff_right": v[101:113],
        "ee_left_marker": v[113:119], "limit_diff_left": v[119:131],
    }


def decode_obs(data):
    """Decode a gateway Obs frame (1060 bytes) into named numpy arrays."""
    v, ts = _payload(data, MSG_OBS, _OBS)
    return _gateway_obs_dict(v), ts


def decode_gateway_obs(v):
    """Name the 131 doubles of priv["gateway_obs"] like decode_obs()."""
    return _gateway_obs_dict(v)


def encode_frame(obs, priv, episode_id, step, frame_state, reason=TR_NONE, reward=0.0, action=(0.0,) * 6,
                 ts=None):
    """obs (OBS_N,), priv (PRIV_N,) float vectors + the tag -> one Frame."""
    if len(obs) != OBS_N or len(priv) != PRIV_N:
        raise ValueError(f"Frame needs obs {OBS_N} and priv {PRIV_N} values, got {len(obs)} / {len(priv)}")
    return _frame(MSG_FRAME, _OBS_V.pack(*obs) + _PRIV_V.pack(*priv)
                  + _TAG.pack(episode_id, step, frame_state, reason, reward, *action), ts)


def decode_frame(data):
    """-> (obs, priv, tag, timestamp). obs / priv: dicts of named numpy arrays (OBS_FIELDS /
    PRIV_FIELDS; size-1 fields as floats) plus "vector" = the whole flat block; obs also has
    joint_pos / joint_vel / joint_effort (25 each) split out of "joints"."""
    import numpy as np
    if len(data) != FRAME_BYTES:
        raise ValueError(f"type {MSG_FRAME}: expected {FRAME_BYTES} bytes, got {len(data)}")
    t, ts = _HEADER.unpack_from(data, 0)
    if t != MSG_FRAME:
        raise ValueError(f"type {t}, expected {MSG_FRAME}")
    o = np.array(_OBS_V.unpack_from(data, _HEADER.size))
    p = np.array(_PRIV_V.unpack_from(data, _HEADER.size + _OBS_V.size))
    v = _TAG.unpack_from(data, _HEADER.size + _OBS_V.size + _PRIV_V.size)

    def named(vec, slices):
        d = {k: (float(vec[a]) if b - a == 1 else vec[a:b]) for k, (a, b) in slices.items()}
        d["vector"] = vec
        return d
    obs, priv = named(o, OBS_SLICES), named(p, PRIV_SLICES)
    j = obs["joints"].reshape(N_JOINT_STATE, 3)
    obs["joint_pos"], obs["joint_vel"], obs["joint_effort"] = j[:, 0], j[:, 1], j[:, 2]
    tag = {"episode_id": v[0], "step": v[1], "frame_state": v[2], "reason": v[3],
           "reward": v[4], "action": np.array(v[5:11])}
    return obs, priv, tag, ts


def layout_markdown():
    """The obs / priv tables as Markdown (FRAME.md is generated from this)."""
    rows = []
    for title, fields, slices in (("obs", OBS_FIELDS, OBS_SLICES), ("priv", PRIV_FIELDS, PRIV_SLICES)):
        rows += [f"### {title} ({sum(n for _, n, _ in fields)} float64)", "",
                 "| index | name | size | meaning |", "|---|---|---|---|"]
        for name, n, doc in fields:
            a, b = slices[name]
            rows.append(f"| {a if n == 1 else f'{a}:{b}'} | `{name}` | {n} | {doc} |")
        rows.append("")
    return "\n".join(rows)
