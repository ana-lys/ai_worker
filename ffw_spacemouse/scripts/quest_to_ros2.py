#!/usr/bin/env python3
"""Controller telemetry → ROS 2 topics for Hand Tracking Streamer (HTS).

Listens for HTS controller packets on UDP or TCP, decodes them, and publishes
at the full incoming rate. Every pose is torso-relative: the origin is the
head's (x, y) projected onto the ground, and the rotation reference is the
head yaw. Until the first head packet arrives, poses are the raw ROS-frame
values.

Topics:
    /left_controller_pose    — PoseStamped, left  Touch controller (torso-relative)
    /right_controller_pose   — PoseStamped, right Touch controller (torso-relative)
    /head_pose               — PoseStamped, head (torso-relative)
    /quest_state             — Float32MultiArray, ALL telemetry (poses, analog,
                               per-button states) as one 38-float array; exact
                               index layout documented below in the
                               "/quest_state Float32MultiArray layout" block.

Examples:
    ros2 run your_package quest_to_ros2.py --protocol udp --port 9500
    python quest_to_ros2.py --protocol tcp --host localhost --port 8500
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import threading
import time
from typing import Iterator, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension
from ffw_spacemouse_msgs.msg import LeftControlOverride


# ---------------------------------------------------------------------------
# /quest_state Float32MultiArray layout — documented contract, do not reorder.
#
# 38 float32 values. Each pose block is 6 floats: position (x, y, z) then
# orientation as roll, pitch, yaw (radians) in the standard ROS convention
# (rotation about fixed X, then fixed Y, then fixed Z == intrinsic ZYX ==
# tf2 getRPY). Poses are torso-relative — the SAME values published on the
# /left_controller_pose, /right_controller_pose, /head_pose PoseStamped topics.
# Analog fields are raw controller-local values (no coordinate transform).
# Each button is its own 0.0/1.0 float (the raw bitmask is decoded here), so
# consumers read button state by index with no bitmask decoding.
#
#   Index  Block / field
#   -----  --------------------------------------------------------------
#   0-5    Left  controller pose  (x, y, z, roll, pitch, yaw)
#   6-9    Left  controller analog (trigger, grip, thumbstick_x, thumbstick_y)
#   10-15  Left  controller buttons (trigger_btn, grip_btn, a_x_btn, b_y_btn,
#                                    thumbstick_btn, menu_btn) — 0.0/1.0 each
#   16-21  Right controller pose  (x, y, z, roll, pitch, yaw)
#   22-25  Right controller analog (trigger, grip, thumbstick_x, thumbstick_y)
#   26-31  Right controller buttons (trigger_btn, grip_btn, a_x_btn, b_y_btn,
#                                    thumbstick_btn, menu_btn) — 0.0/1.0 each
#   32-37  Head pose  (x, y, z, roll, pitch, yaw)
#
# Buttons bitmask bit order (BUTTON_BITS): 0=trigger 1=grip 2=A/X 3=B/Y
#                                          4=thumbstick 5=menu
# ---------------------------------------------------------------------------
PER_POSE_FLOATS = 6         # x, y, z, roll, pitch, yaw
PER_CONTROLLER_FLOATS = 16  # pose (6) + analog (4) + button floats (6)
QUEST_STATE_FLOATS = 38     # left (16) + right (16) + head (6)

LEFT_POSE_BEGIN = 0
RIGHT_POSE_BEGIN = 16
HEAD_POSE_BEGIN = 32

# offsets inside a 6-float pose block
POSE_X, POSE_Y, POSE_Z = 0, 1, 2
POSE_ROLL, POSE_PITCH, POSE_YAW = 3, 4, 5

# offsets inside a controller's 16-float block (relative to *_POSE_BEGIN)
CONTROLLER_TRIGGER_OFFSET = 6
CONTROLLER_GRIP_OFFSET = 7
CONTROLLER_STICK_X_OFFSET = 8
CONTROLLER_STICK_Y_OFFSET = 9
CONTROLLER_TRIGGER_BTN_OFFSET = 10
CONTROLLER_GRIP_BTN_OFFSET = 11
CONTROLLER_AX_BTN_OFFSET = 12
CONTROLLER_BY_BTN_OFFSET = 13
CONTROLLER_THUMBSTICK_BTN_OFFSET = 14
CONTROLLER_MENU_BTN_OFFSET = 15

# Raw buttons bitmask bit order (from HTS): bit N -> the button at
# index N of button_floats() below. Touches are dropped entirely —
# nothing reads them (quest_teleop_plan §3).
BUTTON_BITS = ("trigger", "grip", "a_x", "b_y", "thumbstick", "menu")

# Offset (meters) applied to each controller's tracked origin, expressed in
# the controller's own local frame (ROS convention: X=forward, Y=left,
# Z=up), before publishing. The raw tracked point sits near the front of
# the controller (near the fingers/trigger), so using it directly as an EE
# pose makes rotation pivot there instead of at the wrist. Shifting it along
# the controller's local +X moves the effective pivot toward the wrist.
# Flip the sign if it shifts the wrong way for your grip.
CONTROLLER_EE_OFFSET_LOCAL = (0.1, 0.0, 0.0)


def button_floats(buttons: int) -> list[float]:
    """Decode the buttons bitmask into 6 individual 0.0/1.0 floats.

    Bit order follows BUTTON_BITS: bit 0 -> trigger, bit 1 -> grip,
    bit 2 -> A/X, bit 3 -> B/Y, bit 4 -> thumbstick, bit 5 -> menu.
    """
    return [float((buttons >> i) & 1) for i in range(len(BUTTON_BITS))]


# ----------  network / parsing helpers (adapted from receiver_quest.py)  ----------

def strip_debug_header(label: str) -> str:
    """Drop the ``| f = N | t = NS`` metadata the debug toggle adds to labels."""
    return label.split("|", 1)[0].strip()


def parse_controller_state(values: list[str]):
    """Return all 13 controller fields, or None on malformed input.

    13-field CSV layout::

        0-2    position (px, py, pz)
        3-6    orientation quaternion (qx, qy, qz, qw)
        7-8    trigger, grip        (float 0-1)
        9-10   thumbstick (x, y)    (-1..1)
        11     buttons bitmask      (float on the wire -> int)
        12     touches bitmask      (float on the wire -> int)

    Returns (px, py, pz, qx, qy, qz, qw,
             trigger, grip, stick_x, stick_y, buttons, touches).
    """
    if len(values) < 13:
        return None
    try:
        px, py, pz = (float(v) for v in values[0:3])
        qx, qy, qz, qw = (float(v) for v in values[3:7])
        trigger, grip = float(values[7]), float(values[8])
        stick_x, stick_y = float(values[9]), float(values[10])
        buttons, touches = int(float(values[11])), int(float(values[12]))
    except (ValueError, IndexError):
        return None
    return (px, py, pz, qx, qy, qz, qw,
            trigger, grip, stick_x, stick_y, buttons, touches)


def parse_head_pose(values: list[str]):
    """Return (pos_x, pos_y, pos_z, qx, qy, qz, qw) from a head-pose line.

    Head pose has only 7 fields (position + quaternion), no buttons/etc.
    """
    if len(values) < 7:
        return None
    try:
        px, py, pz = (float(v) for v in values[0:3])
        qx, qy, qz, qw = (float(v) for v in values[3:7])
    except (ValueError, IndexError):
        return None
    return px, py, pz, qx, qy, qz, qw


# ----------  Unity → ROS coordinate conversion  ----------
#
# Unity (left-handed):  X=right,  Y=up,    Z=forward
# ROS   (right-handed): X=forward, Y=left,  Z=up
#
# Position:   ros = (unity_z, -unity_x, unity_y)
# Quaternion: ros = (unity_qz, -unity_qx, unity_qy, unity_qw)

def unity_to_ros_pos(px, py, pz):
    """Convert position from Unity to ROS coordinates."""
    return pz, -px, py


def unity_to_ros_quat(qx, qy, qz, qw):
    """Convert quaternion from Unity to ROS coordinates."""
    return qz, -qx, qy, -qw


def make_pose_stamped(stamp, px, py, pz, qx, qy, qz, qw, frame_id="map"):
    """Build a PoseStamped from Unity-frame pose values (converts to ROS internally)."""
    rx, ry, rz = unity_to_ros_pos(px, py, pz)
    rqx, rqy, rqz, rqw = unity_to_ros_quat(qx, qy, qz, qw)
    msg = PoseStamped()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.pose.position.x = rx
    msg.pose.position.y = ry
    msg.pose.position.z = rz
    msg.pose.orientation.x = rqx
    msg.pose.orientation.y = rqy
    msg.pose.orientation.z = rqz
    msg.pose.orientation.w = rqw
    return msg


# ----------  quaternion helpers (ROS frame, Z-up)  ----------

def yaw_from_quat(qx, qy, qz, qw):
    """Extract yaw (rotation about Z) from a Z-up quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def rpy_from_quat(qx, qy, qz, qw):
    """Extract (roll, pitch, yaw) in radians from a Z-up quaternion.

    Standard ROS convention: rotation about fixed X, then fixed Y, then fixed
    Z (intrinsic ZYX, same as tf2 getRPY). The asin argument is clamped for
    numerical safety at the poles. The yaw term is identical to yaw_from_quat.
    """
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def quat_from_yaw(yaw):
    """Return quaternion (x, y, z, w) for a rotation about Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_mul(q1, q2):
    """Hamilton product of two (x, y, z, w) quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(q, v):
    """Rotate vector v by quaternion q: v' = q v q*."""
    x, y, z = v
    qx, qy, qz, qw = q
    # cross product (q_xyz × v) and double-scalar trick
    ux, uy, uz = qx, qy, qz
    cx, cy, cz = uy * z - uz * y, uz * x - ux * z, ux * y - uy * x
    dot = qx * x + qy * y + qz * z
    s = 2.0
    return (
        x + s * (qw * cx + (uy * cz - uz * cy)),
        y + s * (qw * cy + (uz * cx - ux * cz)),
        z + s * (qw * cz + (ux * cy - uy * cx)),
    )


# ----------  network generators (same as receiver_quest.py)  ----------

def iter_udp_lines(host: str, port: int) -> Iterator[str]:
    """Yield decoded lines from a UDP socket."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    print(f"Listening for UDP on {host}:{port}", file=sys.stderr)
    try:
        while True:
            data, _ = sock.recvfrom(65536)
            for line in data.decode("utf-8", errors="replace").split("\n"):
                if line:
                    yield line
    finally:
        sock.close()


def serve_tcp(host: str, port: int, line_callback) -> None:
    """Accept TCP connections and feed lines to *line_callback*."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(8)
    print(f"Listening for TCP on {host}:{port}", file=sys.stderr)

    def handle_conn(conn: socket.socket, addr):
        print(f"Accepted connection from {addr}", file=sys.stderr)
        buffer = ""
        with conn:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buffer += data.decode("utf-8", errors="replace")
                *lines, buffer = buffer.split("\n")
                for line in lines:
                    if line:
                        line_callback(line)
        print(f"Connection from {addr} closed", file=sys.stderr)

    try:
        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle_conn, args=(conn, addr), daemon=True).start()
    finally:
        server.close()


# ----------  ROS 2 node  ----------

class QuestControllerPoseNode(Node):
    """Listens for HTS controller telemetry and publishes PoseStamped at full rate."""

    def __init__(self, *, tui: bool, bind: str = "udp:0.0.0.0:9500"):
        super().__init__("quest_controller_pose")

        self._pub_left = self.create_publisher(PoseStamped, "left_controller_pose", 10)
        self._pub_right = self.create_publisher(PoseStamped, "right_controller_pose", 10)
        self._pub_head = self.create_publisher(PoseStamped, "head_pose", 10)
        self._pub_state = self.create_publisher(
            Float32MultiArray, "quest_state", 10)
        # Combined left-controller override: right SpaceMouse precision +
        # right gripper velocity + a record-button event, all decoded from
        # the left controller alone (see LeftControlOverride.msg).
        self._pub_left_ctrl = self.create_publisher(
            LeftControlOverride, "/quest/left/control_override", 10)

        # Last-known state per source, kept in step with the PoseStamped topics.
        # Each pose is the torso-relative (pos, quat) value just published;
        # analog is (trigger, grip, stick_x, stick_y, buttons). Touches are
        # dropped (quest_teleop_plan §3 — nothing reads them).
        self._state = {
            "left":  {"pose": None, "analog": None},
            "right": {"pose": None, "analog": None},
            "head":  {"pose": None, "analog": None},
        }

        # Latest head reference (ROS frame, Z-up).
        self._head_pos = None   # (x, y, z) — only x,y are used as origin
        self._head_yaw = 0.0

        # Torso-relative poses for the live HUD.
        self._left_rel = None
        self._right_rel = None
        self._head_rel = None

        # Full-screen (ANSI) 100 Hz HUD, or a throttled plain-line fallback.
        self._tui = tui
        self._bind = bind
        self._last_wait_draw = 0.0
        self._frames = 0
        self._rate_t0 = time.monotonic()
        self._rate = 0.0
        self.create_timer(0.01 if tui else 1.0, self._tick)

    def torso_relative(self, pos, quat):
        """Express *pos*/*quat* relative to the torso base.

        Origin is the head's (x, y) projected onto the ground (x,y → 0,
        height z preserved). Rotation reference is the head yaw only.
        Falls back to raw pose when no head data has arrived yet.
        """
        if self._head_pos is None:
            return pos, quat

        q_inv = quat_from_yaw(-self._head_yaw)
        translated = (pos[0] - self._head_pos[0],
                      pos[1] - self._head_pos[1],
                      pos[2])
        rotated = rotate_vector(q_inv, translated)
        new_quat = quat_mul(q_inv, quat)
        return rotated, new_quat

    def make_torso_pose_stamped(self, stamp, pos, quat):
        """Publish-ready PoseStamped with torso-relative transform applied."""
        rpos, rquat = self.torso_relative(pos, quat)
        msg = PoseStamped()
        msg.header = Header(stamp=stamp, frame_id="map")
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = rpos
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = rquat
        return msg

    def _publish_left_control_override(self, stick_y, trigger, grip, buttons):
        """Decode left thumbstick Y / trigger / grip / A-B buttons into the
        combined override for the right SpaceMouse + gripper + ZMQ record event.
        """
        a_x = (buttons >> BUTTON_BITS.index("a_x")) & 1
        b_y = (buttons >> BUTTON_BITS.index("b_y")) & 1
        msg = LeftControlOverride()
        msg.gripper_vel = float(stick_y)
        msg.trigger = float(trigger)
        msg.grip = float(grip)
        msg.record = (1 if a_x else 0) | (2 if b_y else 0)
        self._pub_left_ctrl.publish(msg)

    def _publish_quest_state(self):
        """Assemble the /quest_state float array from last-known state and publish.

        Layout (documented at the top of this file, do not reorder):
        left pose+analog+buttons (0-15), right pose+analog+buttons (16-31),
        head pose (32-37). Blocks for a source that has not reported yet are
        0.0. Poses are the torso-relative values already published on the
        PoseStamped topics, so /quest_state never disagrees with them. Buttons
        are written as 6 individual 0.0/1.0 floats decoded from the bitmask.
        """
        data = [0.0] * QUEST_STATE_FLOATS

        for name, begin in (("left", LEFT_POSE_BEGIN),
                            ("right", RIGHT_POSE_BEGIN)):
            state = self._state[name]
            if state["pose"] is not None:
                pos, quat = state["pose"]
                data[begin + POSE_X], data[begin + POSE_Y], data[begin + POSE_Z] = pos
                data[begin + POSE_ROLL], data[begin + POSE_PITCH], data[begin + POSE_YAW] = \
                    rpy_from_quat(*quat)
            if state["analog"] is not None:
                trig, grip, sx, sy, btns = state["analog"]
                data[begin + CONTROLLER_TRIGGER_OFFSET] = trig
                data[begin + CONTROLLER_GRIP_OFFSET] = grip
                data[begin + CONTROLLER_STICK_X_OFFSET] = sx
                data[begin + CONTROLLER_STICK_Y_OFFSET] = sy
                btn_begin = begin + CONTROLLER_TRIGGER_BTN_OFFSET
                data[btn_begin:btn_begin + len(BUTTON_BITS)] = button_floats(btns)

        if self._state["head"]["pose"] is not None:
            pos, quat = self._state["head"]["pose"]
            data[HEAD_POSE_BEGIN + POSE_X] = pos[0]
            data[HEAD_POSE_BEGIN + POSE_Y] = pos[1]
            data[HEAD_POSE_BEGIN + POSE_Z] = pos[2]
            data[HEAD_POSE_BEGIN + POSE_ROLL], data[HEAD_POSE_BEGIN + POSE_PITCH], \
                data[HEAD_POSE_BEGIN + POSE_YAW] = rpy_from_quat(*quat)

        msg = Float32MultiArray()
        msg.data = data
        msg.layout.dim = [MultiArrayDimension(label="map", size=QUEST_STATE_FLOATS,
                                              stride=QUEST_STATE_FLOATS)]
        self._pub_state.publish(msg)

    def _fmt_pos(self, p):
        if p is None:
            return "(  ---,   ---,   ---)"
        return f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})"

    def _tick(self):
        """Timer entry point: redraw the HUD, or print a throttled status line."""
        if not self._tui:
            self._print_status()
            return
        # Hold the initial frame (no redraw) until BOTH controllers report in;
        # once they do, redraw every tick (10 ms → ~100 Hz).
        if self._left_rel is None or self._right_rel is None:
            if time.monotonic() - self._last_wait_draw < 1.0:
                return
        self._last_wait_draw = time.monotonic()
        self._draw_tui()

    def _tui_frame(self, rows: list[str]) -> str:
        """Render *rows* as a single ANSI frame — one logical line.

        Homes the cursor, overwrites each row in place (clearing to end of
        line), clears the rest of the screen, then emits one ``\\n``. The
        escape codes are the only "newlines" inside the frame, so
        ``ros2 launch``'s line-based multiplexer emits the whole frame as a
        single unit: the cursor moves back to the top and new data draws over
        old data instead of scrolling the screen.
        """
        parts = ["\x1b[H"]                 # cursor home → top of the HUD block
        for i, row in enumerate(rows):
            if i:
                parts.append("\x1b[E")     # start of next line
            parts.append(row)
            parts.append("\x1b[K")         # clear rest of this line
        parts.append("\x1b[J\n")           # clear rest of screen + flush line
        return "".join(parts)

    def _draw_tui(self):
        """Draw one HUD frame, measuring the achieved refresh rate."""
        self._frames += 1
        now = time.monotonic()
        if now - self._rate_t0 >= 1.0:
            self._rate = self._frames / (now - self._rate_t0)
            self._frames = 0
            self._rate_t0 = now
        rows = [
            f"quest_to_ros2  |  {self._bind}  |  torso-relative XYZ  "
            f"|  {self._rate:6.1f} Hz  (Ctrl+C to quit)",
            "",
            f"  Left  controller  {self._fmt_pos(self._left_rel)}",
            f"  Right controller  {self._fmt_pos(self._right_rel)}",
            f"  Head              {self._fmt_pos(self._head_rel)}",
        ]
        sys.stdout.write(self._tui_frame(rows))
        sys.stdout.flush()

    def _print_status(self):
        """Non-TUI fallback: one plain line per tick, safe to redirect to a file."""
        sys.stdout.write(
            f"Left  {self._fmt_pos(self._left_rel)}   "
            f"Right {self._fmt_pos(self._right_rel)}\n")
        sys.stdout.flush()

    def handle_line(self, line: str) -> None:
        """Parse one telemetry line and publish if it carries controller state."""
        if ":" not in line:
            return

        raw_label, _, payload = line.partition(":")
        label = strip_debug_header(raw_label)

        values = [v.strip() for v in payload.split(",") if v.strip()]
        stamp = self.get_clock().now().to_msg()

        # ---- head pose (7 fields: pos + quat, no buttons/etc) ----
        if label == "Head pose" or label == "Head":
            pose = parse_head_pose(values)
            if pose is None:
                return
            # convert to ROS frame and remember as the torso reference
            rx, ry, rz = unity_to_ros_pos(*pose[0:3])
            rqx, rqy, rqz, rqw = unity_to_ros_quat(*pose[3:7])
            self._head_pos = (rx, ry, rz)
            self._head_yaw = yaw_from_quat(rqx, rqy, rqz, rqw)
            msg = self.make_torso_pose_stamped(stamp, (rx, ry, rz), (rqx, rqy, rqz, rqw))
            self._head_rel = (msg.pose.position.x,
                              msg.pose.position.y,
                              msg.pose.position.z)
            # Keep /quest_state's head block in lock-step with /head_pose.
            self._state["head"] = {
                "pose": ((msg.pose.position.x, msg.pose.position.y,
                          msg.pose.position.z),
                         (msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w)),
                "analog": None,   # head lines carry no analog fields
            }
            self._pub_head.publish(msg)
            self._publish_quest_state()
            return

        # ---- controller state (13 fields) ----
        if "controller" not in label or label.endswith("controller event"):
            return  # skip event lines

        state = parse_controller_state(values)
        if state is None:
            return

        px, py, pz, qx, qy, qz, qw, \
            trigger, grip, stick_x, stick_y, buttons, touches = state
        rx, ry, rz = unity_to_ros_pos(px, py, pz)
        rqx, rqy, rqz, rqw = unity_to_ros_quat(qx, qy, qz, qw)

        # Shift the tracked point forward along the controller's own local
        # axis (see CONTROLLER_EE_OFFSET_LOCAL) before it becomes an EE pose.
        ox, oy, oz = rotate_vector((rqx, rqy, rqz, rqw), CONTROLLER_EE_OFFSET_LOCAL)
        rx, ry, rz = rx + ox, ry + oy, rz + oz

        msg = self.make_torso_pose_stamped(
            stamp, (rx, ry, rz), (rqx, rqy, rqz, rqw))

        # Exact torso-relative pose + raw analog values for /quest_state.
        # Touches are parsed but not stored — /quest_state drops them (§3).
        analog = (trigger, grip, stick_x, stick_y, buttons)
        torso_pos = (msg.pose.position.x, msg.pose.position.y,
                     msg.pose.position.z)
        torso_quat = (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w)

        if "Left" in label or "left" in label:
            self._left_rel = torso_pos
            self._state["left"] = {"pose": (torso_pos, torso_quat),
                                   "analog": analog}
            self._pub_left.publish(msg)
            self._publish_left_control_override(stick_y, trigger, grip, buttons)
            self.get_logger().debug(
                f"L → ({px:.3f}, {py:.3f}, {pz:.3f})  "
                f"({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})",
                throttle_duration_sec=0.25,
            )
            self._publish_quest_state()
        elif "Right" in label or "right" in label:
            self._right_rel = torso_pos
            self._state["right"] = {"pose": (torso_pos, torso_quat),
                                    "analog": analog}
            self._pub_right.publish(msg)
            self.get_logger().debug(
                f"R → ({px:.3f}, {py:.3f}, {pz:.3f})  "
                f"({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})",
                throttle_duration_sec=0.25,
            )
            self._publish_quest_state()


def main():
    parser = argparse.ArgumentParser(
        prog="quest_to_ros2",
        description="Publish HTS controller/head telemetry as ROS 2 messages: "
                    "torso-relative PoseStamped topics plus the /quest_state "
                    "Float32MultiArray (38-float layout documented in the code).",
    )
    parser.add_argument(
        "--protocol", choices=("udp", "tcp"), default="udp",
        help="Transport protocol (default: udp).",
    )
    parser.add_argument(
        "-p", "--port", type=int, default=None,
        help="Port (default: 9500 UDP, 8500 TCP).",
    )
    parser.add_argument(
        "--host", default=None,
        help="Bind address (default: 0.0.0.0 UDP, localhost TCP).",
    )
    parser.add_argument(
        "--tui", action="store_true",
        help="Force the full-screen ANSI 100 Hz HUD even when stdout is not a "
             "terminal (needed under `ros2 launch`/quest.launch.py, which pipes "
             "stdout and would otherwise fall back to line-flooding mode).",
    )
    args = parser.parse_args()

    host = args.host or ("0.0.0.0" if args.protocol == "udp" else "localhost")
    port = args.port or (9500 if args.protocol == "udp" else 8500)
    bind = f"{args.protocol}:{host}:{port}"

    # Full-screen HUD. Auto-enables on a TTY; --tui forces it for the piped
    # stdout that `ros2 launch`/quest.launch.py creates.
    tui = args.tui or sys.stdout.isatty()
    if tui:
        sys.stdout.write("\x1b[?1049h\x1b[2J\x1b[H")   # alt screen, clear, home
        sys.stdout.flush()
    print(f"quest_to_ros2  |  {bind}  |  torso-relative controller XYZ (Ctrl+C to quit)", flush=True)

    rclpy.init()
    node = QuestControllerPoseNode(tui=tui, bind=bind)

    if args.protocol == "udp":

        def udp_spin():
            try:
                for line in iter_udp_lines(host, port):
                    node.handle_line(line)
            except Exception as exc:
                node.get_logger().error(f"UDP listener error: {exc}")
                rclpy.shutdown()

        import threading as _t
        net_thread = _t.Thread(target=udp_spin, daemon=True)
        net_thread.start()

        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            # SIGINT: rclpy may surface it as KeyboardInterrupt (raised in the
            # main thread) or ExternalShutdownException (the context is already
            # shut down when the executor wakes). Either way, exit cleanly.
            pass
    else:
        # TCP — serve_tcp is blocking, so run rclpy.spin in a thread
        def ros_spin():
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass

        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()

        try:
            serve_tcp(host, port, node.handle_line)
        except KeyboardInterrupt:
            pass

    if tui:
        # Clear the alt screen, then restore the main screen so the user's
        # terminal content comes back instead of leaving a mangled HUD.
        sys.stdout.write("\x1b[2J\x1b[H\x1b[?1049l")
        sys.stdout.flush()
    print(flush=True)  # leave a fresh line so the shell prompt doesn't clobber the HUD
    # SIGINT makes rclpy shut the context down itself, so only tear down if
    # the context is still live (avoids "rcl_shutdown already called").
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
