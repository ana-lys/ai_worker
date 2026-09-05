"""Gateway between ZMQ (RL/controller side) and the local spacemouse teleop stack.

Runs on the machine attached to the robot. Bridges two worlds:

  ZMQ side (same HIL link as robot_node, so they are mutually exclusive):
    PUB tcp://*:STATE_PORT   -- Obs (current dual EE poses + grippers + joints)
    PUB tcp://*:PRIV_PORT    -- Priv (head-cam tf + per-tick spacemouse deltas)
    PUB tcp://*:RECORD_PORT  -- Record (left A/X + B/Y button event, on change)
    SUB tcp://*:CONTROL_PORT -- ControlCmd (EE command) + OverrideCmd
                                 (joint-space command) from the controller

  ROS side (spacemouse teleop stack):
    sub /ik_solver/achieved_ee_pose_{r,l}  -- current achieved EE poses (map frame)
    pub /quest/{right,left}/ee_target_pose -- absolute goal poses for the solver
    pub /quest/{right,left}/trigger       -- gripper command (0.0 open .. 1.0 close)
    sub /joint_states                     -- current joints (gripper feedback +
                                             the Obs joint block)
    sub /spacemouse/{right,left}/ee_target_delta -- per-tick spacemouse deltas
                                             (deltas out -> the Priv EEDelta block)
    sub /quest/left/control_override      -- left-controller override (see
                                             ffw_spacemouse_msgs/LeftControlOverride);
                                             only .record is relayed, on RECORD_PORT

ee0 <-> right arm, ee1 <-> left arm (see protocol.py).

Role split (leRobot-style, protocol.py): Obs on STATE_PORT is one complete,
self-contained observation per tick -- the receiver syncs and concats ticks
without cross-referencing separate streams. Priv on PRIV_PORT is privileged
context NOT fed to a learned network: the head-camera tf for vision consumers
CONCAT the per-tick commanded deltas (the expert label for demos / a future
delta-space action rail). Action: ControlCmd (EE command) + OverrideCmd
(joint-space command) on CONTROL_PORT.

Obs cadence: the EE block is emitted only once BOTH arms have an achieved pose
AND the first /joint_states has arrived (a zero-filled joint block would be a
worse bug than silence); after that the frame is sent EVERY tick at 100 Hz,
repeating the newest joint block between /joint_states broadcasts (~50 Hz). The
frame is one self-contained observation (joint block leads, EE block at the
tail -- protocol.py layout), so repeating the newest joint block is always
correct.

Priv cadence: the head-cam tf block is emitted only once the bridge has
resolved it (odom/head joints up -- the identity is never sent as a real
transform); after that the frame is sent EVERY tick. The EEDelta block inside
is per-arm fresh-windowed: an arm is relayed only while its delta stream is
fresh (a message arrived within _DELTA_FRESH_S since the last relay) and is
zero-filled otherwise (all-zeros == "no command this tick"). Because the frame
is always sent once the tf is known, deltas arriving before the tf resolves
are dropped -- acceptable: no demo/tf consumer can act until the tf exists.

The command path feeds /quest/<arm>/ee_target_pose, the absolute-gesture path in
ffw_ik_solver_teleop. That path consumes a map-frame goal directly (leashed
~6 cm/tick), is NOT gated on the quest being active, and is the correct injection
point for remote commands -- the delta path /spacemouse/<arm>/ee_target_pose
treats a large jump as a mapper re-base and would swallow a distant command.

The gripper value rides /quest/<arm>/trigger, the same channel the physical
spacemouse quest trigger drives. It maps to the solver's gripper joints via
update_grippers() and, like the physical trigger, is obeyed only while the quest
is in TRACK -- during CTRL/APPROACH the trigger feed is ignored (the 4-way
engage hold must not slam the gripper). The pose path is ungated; the gripper
path is TRACK-gated. Obs carries the gripper back to the controller, normalized
from /joint_states with the solver's own open/closed constants
(_GRIP_OPEN/_GRIP_CLOSED below) so command -> feedback is round-trip symmetric.

OverrideCmd (MSG_OVERRIDE, protocol.py): the controller's joint-space command
rail -- the exact 25 commanded joint positions in JOINT_STATE_NAMES order, the
alternative to the EE-space ControlCmd. It is not a latch and carries no 0/1
engage bit: the command stream itself is the control signal. This gateway
decodes and holds the newest OverrideCmd, and relays it to /qpos_rail
(sensor_msgs/JointState) every tick while fresh (staleness gated by
--cmd-timeout, same as ControlCmd). ffw_ik_solver_teleop runs goal_source=
ee|rail and apply_rail_sync() on /qpos_rail ticks (commit 17a6a60) and rejects
a message unless it names every joint its MuJoCo model expects with no
unrecognized names (gripper_l_joint1 is the one allowed exception -- it has no
MuJoCo joint either). The model has no mobile-base wheel joints at all, so the
relay drops the 6 *_wheel_drive/*_wheel_steer names from the 25 (_RAIL_EXCLUDE)
and forwards every other joint (arms, grippers, head, lift) by name. The old
/control_override Bool latch and its joy_hand force-TRACK coupling are
retired.

Loop integrity: this gateway is the only process that sees BOTH the Obs it
sends out (the joint block on STATE_PORT) and the OverrideCmd it receives back
(CONTROL_PORT), so "did what came back match what we sent" is answerable only
here, not on any controller side: a client can see only what it sent back
versus the Obs it received, never both halves against one reference. The
companion echo generator (examples/override_round_trip_check.py) exists to
drive this check; its own table is a transport/ordering read from the client
side of the link. With override_check (default on), every
received OverrideCmd is diffed per-joint against the joint block of the last
Obs the gateway published; any frame past override_margin is WARNed, a running
cumulative summary is INFO-logged every override_report_s seconds while
overrides arrive, and a full per-joint table + verdict is logged at shutdown.
A controller that echoes the Obs joint block back as OverrideCmd (the sustained
relay check) gives diff ~ 0 with the robot holding still -- nonzero means the
25 floats were corrupted / reordered on the link, or another commander moved
the robot. A real joint-space rail is a tracking command, not an echo, so there
a nonzero diff is command-vs-current distance, not a transport error.

Every ControlCmd frame carries a sender timestamp in its header (protocol.py),
so re-asserting the same goal is byte-unique and passes the _last_pub_data
dedup -- the dedup only suppresses exact retransmissions of one frame. A
controller walking the solver's per-message leash (e.g. a 25 Hz re-assert loop)
gets every re-assertion forwarded, no payload jitter required.

When the controller stops sending, the gateway stops publishing after
--cmd-timeout and the solver holds its last quest goal. Moving the spacemouse
re-engages the delta path (joy_hand publishes /spacemouse/<arm>/ee_target_pose).

ZMQ note: both gateway PUBs stay non-CONFLATE. Each carries one message type
per tick at full rate, and the tf batch consumers (BoardTfLink) must be able to
drain every frame to detect a head-still board. A consumer that only wants the
newest frame may CONFLATE its own SUB side (per-socket, never across the link).

Usage:
    ros2 run ffw_zmqinterface gateway_node --ros-args \
        -p state_port:=6001 -p priv_port:=6003 -p control_port:=6002 -p hz:=100.0 \
        -p override_check:=true -p override_margin:=0.02 -p override_report_s:=10.0
"""

import math
import os
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from ffw_spacemouse_msgs.msg import LeftControlOverride

import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as proto


def _declare_float(node, name, default):
    """Declare a float param that also accepts integer overrides (e.g. -p hz:=100)."""
    try:
        node.declare_parameter(name, default)
    except rclpy.exceptions.InvalidParameterTypeException:
        node.declare_parameter(name, int(default))
    return float(node.get_parameter(name).value)


# protocol ee index -> arm/topic. ee0 = right, ee1 = left; flip the tuple to swap.
_CMD_TOPIC = {0: "/quest/right/ee_target_pose", 1: "/quest/left/ee_target_pose"}
_ACHIEVED_TOPIC = {0: "/ik_solver/achieved_ee_pose_r", 1: "/ik_solver/achieved_ee_pose_l"}
# Per-tick spacemouse commanded deltas (deltas out), relayed as the Priv EEDelta block.
_DELTA_TOPIC = {0: "/spacemouse/right/ee_target_delta", 1: "/spacemouse/left/ee_target_delta"}
# An arm's delta stream is relayed only while a message has arrived within this
# window of the last relay; otherwise the arm is zero-filled on the wire.
_DELTA_FRESH_S = 0.25
# Gripper command channel: the same /quest/<arm>/trigger the physical spacemouse
# quest trigger drives (joy_hand). Float32, 0.0 = open .. 1.0 = close.
_GRIP_TOPIC = {0: "/quest/right/trigger", 1: "/quest/left/trigger"}
# head_camera_tf_bridge republishes the resolved camera->control transform here;
# the gateway forwards it as the Priv HeadCamTf block.
_CAM_TF_TOPIC = "/head_camera_tf"
# Current gripper joints, for Obs EE-block feedback. Sensor_msgs/JointState.
_JOINT_STATES_TOPIC = "/joint_states"
# OverrideCmd -> the IK solver's qpos rail (ffw_ik_solver_teleop, mode B).
_RAIL_TOPIC = "/qpos_rail"
# Left Meta Quest controller override (quest_to_ros2.py): only .record is
# relayed here, as a Record event on RECORD_PORT (see protocol.py).
_LEFT_CTRL_TOPIC = "/quest/left/control_override"
# Joints in JOINT_STATE_NAMES with no counterpart in the IK solver's MuJoCo
# model (the mobile-base wheel drives/steers -- the model only covers the
# arms/grippers/head/lift). apply_rail_sync rejects the WHOLE message if it
# contains any name it doesn't recognize (gripper_l_joint1 is the one allowed
# exception), so these must be dropped before forwarding, not just zeroed.
_RAIL_EXCLUDE = frozenset(n for n in proto.JOINT_STATE_NAMES if "wheel" in n)
# OverrideCmd loop-integrity check: how many out-of-margin WARN lines to log
# before going quiet and letting the periodic summary count the rest.
_OVR_WARN_LIMIT = 3
_GRIP_JOINT = {0: "gripper_r_joint1", 1: "gripper_l_joint1"}

# Gripper [0,1] normalization bounds per arm -- the solver's own constants for
# ffw_sg2_smtm (ffw_ik_solver_teleop.cpp kGripperRMin/kGripperRMax,
# kGripperLMin/kGripperLMax; "larger = closed"). Using the same values keeps
# command -> Obs EE feedback round-trip symmetric: 0.0 = open, 1.0 = closed.
# Model-specific, not wire contract. Left-closed is 1.2 on the wire but the
# left URDF caps at 1.1 and the controller clamps, so settled-closed may read
# ~0.92 -- adjust here if live feedback disagrees.
_GRIP_OPEN = {0: 0.175, 1: 0.0}
_GRIP_CLOSED = {0: 1.2, 1: 1.2}


def _normalize_grip(radians, idx):
    """Joint angle (rad) -> [0, 1] gripper value using the solver's bounds.

    0.0 = open, 1.0 = closed. Returns 0.0 if the joint is not on the wire.
    """
    if radians is None:
        return 0.0
    lo, hi = _GRIP_OPEN[idx], _GRIP_CLOSED[idx]
    return proto.clamp01((radians - lo) / (hi - lo))


def _reorder_joint_block(js_msg):
    """Reindex a sensor_msgs/JointState into JOINT_STATE_NAMES order.

    Returns (pos, vel, eff), three lists of length N_JOINT_STATE, 0.0-filled
    for any joint missing from the message (broadcasters may omit joints).
    """
    pos = dict(zip(js_msg.name, js_msg.position))
    vel = dict(zip(js_msg.name, js_msg.velocity))
    eff = dict(zip(js_msg.name, js_msg.effort))
    return (
        [pos.get(name, 0.0) for name in proto.JOINT_STATE_NAMES],
        [vel.get(name, 0.0) for name in proto.JOINT_STATE_NAMES],
        [eff.get(name, 0.0) for name in proto.JOINT_STATE_NAMES],
    )


def _matrix_from_transform(msg):
    """geometry_msgs/TransformStamped -> row-major 4x4 homogeneous (16 floats).

    R is the standard quaternion->matrix (w,x,y,z), translation in column 3.
    Matches protocol.py Priv matrix: p_control = M * [p_camera, 1].
    """
    q = msg.transform.rotation
    t = msg.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    return (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
            2.0 * (x * z + y * w), t.x,
            2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w), t.y,
            2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y), t.z,
            0.0, 0.0, 0.0, 1.0)


class GatewayNode(Node):
    def __init__(self):
        super().__init__("zmq_spacemouse_gateway")
        self.declare_parameter("state_port", 6001)
        self.declare_parameter("priv_port", 6003)
        self.declare_parameter("control_port", 6002)
        self.declare_parameter("record_port", 6004)
        hz = _declare_float(self, "hz", 100.0)
        self._cmd_timeout = _declare_float(self, "cmd_timeout", 0.5)  # s; stop forwarding after this gap
        # OverrideCmd loop-integrity check (see the module docstring).
        self.declare_parameter("override_check", True)   # diff received overrides vs last sent Obs
        self._override_margin = _declare_float(self, "override_margin", 0.02)  # rad; past this = WARN
        self._override_report_s = _declare_float(self, "override_report_s", 10.0)  # s between summaries
        self._override_check = bool(self.get_parameter("override_check").value)
        state_port = self.get_parameter("state_port").value
        priv_port = self.get_parameter("priv_port").value
        control_port = self.get_parameter("control_port").value
        record_port = self.get_parameter("record_port").value

        # Latest data, shared with the ZMQ receive thread. Guarded by _lock.
        self._lock = threading.Lock()
        self._achieved = [None, None]  # index 0=right(ee0), 1=left(ee1)
        self._cmd = None               # newest ControlCmd
        self._cmd_ts = 0.0             # sender timestamp from the frame header
        self._cmd_recv_ts = 0.0        # monotonic time it arrived (timeout basis)
        self._last_pub_data = None     # dedup: last full frame bytes we forwarded
        self._cam_tf = None            # newest head-camera transform (TransformStamped)
        self._joint_state = None       # newest /joint_states (gripper feedback + Obs joint block)
        self._override_cmd = None      # newest OverrideCmd (joint-space rail), held for the solver side
        self._override_ts = 0.0        # monotonic time it arrived (liveness for a future relay)
        # OverrideCmd loop-integrity state (guarded by _lock). The reference for
        # "is the received override close to the Obs we sent out" is the joint
        # block of the LAST Obs actually published (_tick snapshots it at the
        # encode/send site, so it is exactly what was on the wire).
        self._sent_joint_pos = None    # the 25 joint_pos in the last Obs we published
        self._sent_ts = 0.0            # monotonic time of that send
        self._ovr_stats = {
            "frames": 0,               # OverrideCmd frames compared vs sent Obs
            "over": 0,                 # frames with any |diff| > override_margin
            "over_warned": 0,          # WARN lines already emitted (throttled)
            "max_abs": [0.0] * proto.N_JOINT_STATE,  # per-joint max |cmd - sent|
            "last": [0.0] * proto.N_JOINT_STATE,      # per-joint diff on the newest frame
            "report_ts": 0.0,          # monotonic: last periodic INFO summary
        }
        # Spacemouse per-tick commanded deltas (index 0=right/ee0, 1=left/ee1).
        self._delta = [None, None]       # newest TwistStamped per arm
        self._delta_recv = [0.0, 0.0]    # monotonic time it arrived
        self._delta_sent = [0.0, 0.0]    # monotonic time of the last relay

        # Record event (left controller A/X + B/Y): sent only on change, so
        # this is the last value actually relayed, not a per-tick snapshot.
        self._last_record = 0

        # ZMQ: Obs out (PUB on STATE), Priv out (PUB on PRIV), control in (SUB
        # on CONTROL). No ZMQ_CONFLATE: each PUB carries one message type per
        # tick at full rate and the tf-batch consumers must be able to drain
        # every frame (a conflated socket would drop the older tf frames used
        # for head-still detection). Consumers that want newest-only can set
        # CONFLATE on their own SUB side.
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{state_port}")
        self._priv_pub = self._ctx.socket(zmq.PUB)
        self._priv_pub.bind(f"tcp://*:{priv_port}")
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.bind(f"tcp://*:{control_port}")
        self._record_pub = self._ctx.socket(zmq.PUB)
        self._record_pub.bind(f"tcp://*:{record_port}")

        # ROS: current poses in, goal poses + gripper out, camera transform in.
        for i, topic in _ACHIEVED_TOPIC.items():
            self.create_subscription(
                PoseStamped, topic, lambda msg, idx=i: self._on_achieved(idx, msg), 10
            )
        self.create_subscription(
            TransformStamped, _CAM_TF_TOPIC, self._on_cam_tf, 10
        )
        self.create_subscription(JointState, _JOINT_STATES_TOPIC, self._on_joint_state, 10)
        for i, topic in _DELTA_TOPIC.items():
            self.create_subscription(
                TwistStamped, topic, lambda msg, idx=i: self._on_delta(idx, msg), 10
            )
        self.create_subscription(
            LeftControlOverride, _LEFT_CTRL_TOPIC, self._on_left_ctrl, 10
        )
        self._quest_pubs = {
            i: self.create_publisher(PoseStamped, topic, 10)
            for i, topic in _CMD_TOPIC.items()
        }
        self._trigger_pubs = {
            i: self.create_publisher(Float32, topic, 10)
            for i, topic in _GRIP_TOPIC.items()
        }
        self._rail_pub = self.create_publisher(JointState, _RAIL_TOPIC, 10)
        self._recv_thread = threading.Thread(
            target=self._recv_loop, name="zmq-control-recv", daemon=True
        )
        self._recv_thread.start()

        period = 1.0 / hz
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"gateway: Obs-> {state_port}, Priv-> {priv_port}, "
            f"Record-> {record_port} (PUB), "
            f"ControlCmd/OverrideCmd<- {control_port} (SUB) -> "
            f"{_RAIL_TOPIC} (rail), {hz:.0f} Hz, "
            f"cmd timeout {self._cmd_timeout:.1f} s"
        )

    # -- receive thread: ZMQ SUB only --------------------------------

    def _recv_loop(self):
        while True:
            try:
                data = self._sub.recv()
            except zmq.ZMQError:
                break
            try:
                msg_type = proto._HEADER.unpack_from(data, 0)[0]
            except struct.error:
                self.get_logger().warn("dropped undersized frame")
                continue
            if msg_type == proto.MSG_CONTROL_CMD:
                try:
                    cmd, ts = proto.decode_control(data)
                except ValueError:
                    self.get_logger().warn("dropped malformed ControlCmd")
                    continue
                with self._lock:
                    self._cmd = cmd
                    self._cmd_ts = ts          # sender clock, kept for re-encode
                    self._cmd_recv_ts = time.monotonic()
            elif msg_type == proto.MSG_OVERRIDE:
                try:
                    ovr, _ts = proto.decode_override(data)
                except ValueError:
                    self.get_logger().warn("dropped malformed OverrideCmd")
                    continue
                self._on_override(ovr)
            else:
                self.get_logger().warn(f"dropped frame with unknown type {msg_type}")

    # -- executor thread: ROS subs + timer only ----------------------

    def _on_achieved(self, idx, msg):
        with self._lock:
            self._achieved[idx] = msg

    def _on_cam_tf(self, msg):
        with self._lock:
            self._cam_tf = msg

    def _on_joint_state(self, msg):
        with self._lock:
            self._joint_state = msg

    def _on_delta(self, idx, msg):
        with self._lock:
            self._delta[idx] = msg
            self._delta_recv[idx] = time.monotonic()

    def _on_left_ctrl(self, msg):
        # Record is a discrete event (unlike Obs/Priv): relay only on change,
        # from this callback rather than the fixed-cadence _tick.
        record = int(msg.record)
        with self._lock:
            changed = record != self._last_record
            self._last_record = record
        if changed:
            self._record_pub.send(proto.encode_record(proto.Record(record)))

    def _tick(self):
        with self._lock:
            achieved = list(self._achieved)
            cmd = self._cmd
            cmd_ts = self._cmd_ts
            cmd_recv_ts = self._cmd_recv_ts
            cam_tf = self._cam_tf
            joint_state = self._joint_state
            delta = list(self._delta)
            delta_recv = list(self._delta_recv)
            delta_sent = list(self._delta_sent)
            override_cmd = self._override_cmd
            override_ts = self._override_ts
        now = time.monotonic()

        # Obs out: one self-contained observation per tick -- dual EE poses
        # (quat -> rpy on the wire) + grippers + the full joint block in
        # JOINT_STATE_NAMES order. Gated on BOTH arms achieved AND the first
        # /joint_states (a zero-filled joint block would be worse than
        # silence); once resolved it is sent every tick at hz, repeating the
        # newest joint block between /joint_states broadcasts.
        if (achieved[0] is not None and achieved[1] is not None
                and joint_state is not None):
            ee = []
            grip = [0.0, 0.0]
            pos = dict(zip(joint_state.name, joint_state.position))
            for i, msg in enumerate(achieved):
                p = msg.pose
                rx, ry, rz = proto.quat_to_rpy(
                    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
                )
                ee.append((p.position.x, p.position.y, p.position.z, rx, ry, rz))
                grip[i] = _normalize_grip(pos.get(_GRIP_JOINT[i]), i)
            if all(math.isfinite(v) for arm in ee for v in arm):
                obs = proto.Obs(tuple(ee), gripper=(grip[0], grip[1]))
                (obs.joint_pos, obs.joint_vel,
                 obs.joint_effort) = _reorder_joint_block(joint_state)
                self._pub.send(proto.encode_obs(obs))
                # Loop-integrity reference: snapshot the joint block that is
                # actually on the wire (obs.joint_pos, not the raw /joint_states)
                # so received OverrideCmds are compared against exactly what we
                # sent out.
                with self._lock:
                    self._sent_joint_pos = list(obs.joint_pos)
                    self._sent_ts = time.monotonic()

        # Priv out: head-cam tf + per-tick commanded deltas. The tf block is
        # only sent once the bridge has resolved it (odom/head joints up); the
        # delta block is per-arm fresh-windowed and zero-filled otherwise.
        # Frame is sent every tick once the tf is known -> fixed cadence.
        if cam_tf is not None:
            vals = [[0.0] * 6, [0.0] * 6]
            for i in (0, 1):
                if (delta[i] is not None and delta_recv[i] > delta_sent[i]
                        and (now - delta_recv[i]) < _DELTA_FRESH_S):
                    t = delta[i].twist
                    vals[i] = [t.linear.x, t.linear.y, t.linear.z,
                               t.angular.x, t.angular.y, t.angular.z]
            self._priv_pub.send(proto.encode_priv(
                proto.Priv(matrix=_matrix_from_transform(cam_tf),
                           delta=(tuple(vals[0]), tuple(vals[1])))))
            with self._lock:
                for i in (0, 1):
                    if delta_recv[i] > delta_sent[i]:
                        self._delta_sent[i] = time.monotonic()

        # Command in: newest ControlCmd -> /quest/<arm>/ee_target_pose.
        if cmd is not None and (now - cmd_recv_ts) < self._cmd_timeout:
            self._forward_cmd(cmd, cmd_ts)

        # Command in: newest OverrideCmd -> /qpos_rail (joint-space rail).
        # Not a latch (protocol.py) -- staleness is treated like ControlCmd.
        if override_cmd is not None and (now - override_ts) < self._cmd_timeout:
            self._forward_override(override_cmd)

    def _forward_cmd(self, cmd, ts):
        # Finiteness check covers poses and gripper values (both go on the wire).
        vals = cmd.ee[0] + (cmd.gripper[0],) + cmd.ee[1] + (cmd.gripper[1],)
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().warn("dropped non-finite ControlCmd")
            return
        data = proto.encode_control(cmd, ts=ts)
        if data == self._last_pub_data:
            return  # identical frame (type+ts+payload) already forwarded
        for i in (0, 1):
            x, y, z, rx, ry, rz = cmd.ee[i]
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            w, qx, qy, qz = proto.rpy_to_quat(rx, ry, rz)
            pose.pose.orientation.w = w
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            self._quest_pubs[i].publish(pose)
            # Gripper rides the same trigger channel as the physical spacemouse
            # quest trigger (0 = open, 1 = close), clamped to [0,1]. Obeyed by
            # the solver only while the quest is in TRACK -- like the trigger.
            trigger = Float32()
            trigger.data = proto.clamp01(cmd.gripper[i])
            self._trigger_pubs[i].publish(trigger)
        self._last_pub_data = data

    def _forward_override(self, ovr):
        """Relay the newest OverrideCmd to /qpos_rail, dropping the wheel
        joints apply_rail_sync doesn't recognize (see _RAIL_EXCLUDE) -- every
        other joint (arms, grippers, head, lift) is forwarded by name so the
        solver's name->qposadr match on its side lines up 1:1.
        """
        if not all(math.isfinite(v) for v in ovr.joint_pos):
            self.get_logger().warn("dropped non-finite OverrideCmd")
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, pos in zip(proto.JOINT_STATE_NAMES, ovr.joint_pos):
            if name in _RAIL_EXCLUDE:
                continue
            msg.name.append(name)
            msg.position.append(pos)
        self._rail_pub.publish(msg)

    # -- OverrideCmd loop-integrity check ------------------------------

    def _on_override(self, ovr):
        """Store the newest OverrideCmd and, when the check is on, compare it
        against the joint block of the last Obs we sent out.

        Only the gateway sees both rails, so this is where "is the received-back
        OverrideCmd close to the Obs the gateway sent" is answered. Called from
        the ZMQ receive thread. Logging happens outside _lock.
        """
        now = time.monotonic()
        n = proto.N_JOINT_STATE
        margin = self._override_margin
        st = self._ovr_stats
        log = None  # ("warn" | "info", text)
        with self._lock:
            self._override_cmd = ovr
            self._override_ts = now
            ref = self._sent_joint_pos
            if not self._override_check or ref is None:
                return  # check disabled, or no Obs sent yet (nothing to compare)
            st["frames"] += 1
            over = []
            worst = 0.0
            for i in range(n):
                d = ovr.joint_pos[i] - ref[i]
                st["last"][i] = d
                ad = abs(d)
                if ad > st["max_abs"][i]:
                    st["max_abs"][i] = ad
                if ad > worst:
                    worst = ad
                if ad > margin:
                    over.append(i)
            if over:
                st["over"] += 1
                if st["over_warned"] < _OVR_WARN_LIMIT:
                    st["over_warned"] += 1
                    names = ", ".join(proto.JOINT_STATE_NAMES[i]
                                      for i in over[:_OVR_WARN_LIMIT + 3])
                    if len(over) > _OVR_WARN_LIMIT + 3:
                        names += ", ..."
                    log = ("warn",
                           f"received OverrideCmd is {worst:.4f} from the Obs "
                           f"joint block we sent out (margin {margin:.4f}); "
                           f"over on: {names}")
            if log is None and now - st["report_ts"] >= self._override_report_s:
                st["report_ts"] = now
                log = ("info", self._override_line(st))
        if log is not None:
            # Two literal call sites, not getattr(logger, name)(...) from one --
            # rclpy's logger keys its per-call-site cache on (file, line), so
            # dispatching both severities through a single shared call site
            # raises "Logger severity cannot be changed between calls" the
            # first time the severity differs from the prior call, killing
            # this thread (and with it all further OverrideCmd/ControlCmd
            # processing -- the gateway looked like it "moved once then died").
            text = f"[override-loop] {log[1]}"
            if log[0] == "warn":
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)

    def _override_line(self, st):
        """One-line cumulative summary for the periodic [override-loop] log."""
        worst = max(st["max_abs"])
        name = proto.JOINT_STATE_NAMES[st["max_abs"].index(worst)]
        return (f"{st['frames']} frame(s) checked against the Obs joint block "
                f"the gateway sent out: max|cmd-obs| = {worst:.4f} rad "
                f"({name}), {st['over']} over margin "
                f"{self._override_margin:.4f}")

    def _print_override_summary(self):
        """Log a full per-joint table + verdict (called at shutdown)."""
        st = self._ovr_stats
        with self._lock:
            frames = st["frames"]
            if frames == 0:
                return
            margin = self._override_margin
            max_abs = list(st["max_abs"])
            last = list(st["last"])
            over = st["over"]
        names = proto.JOINT_STATE_NAMES
        log = self.get_logger()
        log.info(f"[override-loop] {frames} OverrideCmd frame(s) compared "
                 f"against the Obs joint block the gateway sent out "
                 f"(margin {margin:.4f}): {over} frame(s) over margin")
        log.info(f"[override-loop]   {'idx':>3} {'joint':<20} "
                 f"{'max|cmd-obs|':>12} {'last diff':>10}")
        over_joints = 0
        for i in range(len(names)):
            ok = max_abs[i] <= margin
            if not ok:
                over_joints += 1
            log.info(f"[override-loop]   {i:>3} {names[i]:<20} "
                     f"{max_abs[i]:12.4f} {last[i]:+10.4f} "
                     f"{'ok' if ok else 'OVER'}")
        if over == 0:
            verdict = (f"PASS -- every received OverrideCmd stayed within "
                       f"{margin:.4f} of the Obs joint block we sent out")
        else:
            verdict = (f"FAIL -- {over} frame(s) past the margin "
                       f"({over_joints} joint(s) over); link corrupted or "
                       f"another commander moved the robot")
        log.info(f"[override-loop] verdict: {verdict}")


def main():
    rclpy.init()
    node = GatewayNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._print_override_summary()  # best-effort; never mask the exit cause
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
