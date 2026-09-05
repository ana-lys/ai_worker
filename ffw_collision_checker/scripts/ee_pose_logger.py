#!/usr/bin/env python3
"""Named EE-pose snapshot logger.

Live readout of the left/right achieved EE pose (xyz + rpy) on /ik_solver/
achieved_ee_pose_l|r, redrawn in place so it never floods the screen.

The name field is always live - no Enter needed to start editing: type to
build the name, backspace to fix it. Press Enter to save the CURRENT pose
(captured at that Enter press) as one JSON object appended to the snapshot
file. The name is NOT cleared after a save - backspace over it to change it.
Empty name = auto 'snap_HHMMSS'. The 'last pose name' line updates after
every save as confirmation.
  Ctrl-C -> quit.

Each saved JSON object: {name, wall_time, saved_at, ee_l, ee_r, joints}.
The EE entries carry xyz + quat + rpy (extrinsic-XYZ, protocol.py convention).
joints is the raw /joint_states float arrays (names/position/velocity), or
null when no joint-state publisher is up (sim-only mode).

    cd ~/robotis_ws && source install/setup.bash
    python3 src/ai_worker/ffw_collision_checker/scripts/ee_pose_logger.py
"""

import argparse
import json
import math
import os
import select
import shutil
import sys
import termios
import time
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

EE_TOPIC_L = "/ik_solver/achieved_ee_pose_l"
EE_TOPIC_R = "/ik_solver/achieved_ee_pose_r"
JOINT_TOPIC = "/joint_states"

# After a handled Enter some terminals deliver a trailing \r/\n echo. That byte
# only ever arrives within a few ms of the press, so any \r/\n that shows up
# within ENTER_ECHO_WINDOW of the Enter we acted on is dropped as that press's
# echo. A later, deliberate Enter is never swallowed.
ENTER_ECHO_WINDOW = 0.12      # seconds


def quat_to_rpy(x, y, z, w):
    """Radians; same extrinsic-XYZ convention as ffw_zmqinterface/protocol.py."""
    roll = math.atan2(2.0 * (w * x - y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y + x * z))))
    yaw = math.atan2(2.0 * (w * z - x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def ros_stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class EEPoseLogger(Node):
    """Caches the newest message per topic; nothing else."""

    def __init__(self):
        super().__init__("ee_pose_logger")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE,
                         depth=1)
        self.latest = {"ee_l": None, "ee_r": None, "joints": None}
        self.sub_ee_l = self.create_subscription(PoseStamped, EE_TOPIC_L,
                                                 self._on_ee_l, qos)
        self.sub_ee_r = self.create_subscription(PoseStamped, EE_TOPIC_R,
                                                 self._on_ee_r, qos)
        self.sub_js = self.create_subscription(JointState, JOINT_TOPIC,
                                               self._on_joints, qos)

    def _on_ee_l(self, msg):
        self.latest["ee_l"] = msg

    def _on_ee_r(self, msg):
        self.latest["ee_r"] = msg

    def _on_joints(self, msg):
        self.latest["joints"] = msg


def ee_to_dict(msg):
    if msg is None:
        return None
    p, q = msg.pose.position, msg.pose.orientation
    r, pi, y = quat_to_rpy(q.x, q.y, q.z, q.w)
    return {
        "stamp": round(ros_stamp_sec(msg), 6),
        "frame": msg.header.frame_id,
        "pos": [round(p.x, 6), round(p.y, 6), round(p.z, 6)],
        "quat": [round(q.x, 6), round(q.y, 6), round(q.z, 6), round(q.w, 6)],
        "rpy_rad": [round(r, 6), round(pi, 6), round(y, 6)],
        "rpy_deg": [round(math.degrees(r), 3), round(math.degrees(pi), 3),
                    round(math.degrees(y), 3)],
    }


def joints_to_dict(msg):
    if msg is None:
        return None
    return {
        "stamp": round(ros_stamp_sec(msg), 6),
        "names": list(msg.name),
        "position": [round(float(v), 6) for v in msg.position],
        "velocity": ([round(float(v), 6) for v in msg.velocity]
                     if msg.velocity else None),
    }


class Terminal:
    """Raw-mode stdin + in-place redraw of a screen region.

    Raw mode disables output post-processing, so a plain '\\n' moves the cursor
    down WITHOUT returning to column 0 (that was the flood bug). Every newline
    here is '\\r\\n'. Each redraw returns to the region's top line, erases to
    the end of the screen, and rewrites — the region self-corrects even if its
    height changes and never scrolls the terminal.
    """

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved_attrs = termios.tcgetattr(self.fd)
        self.is_tty = sys.stdin.isatty() and sys.stdout.isatty()
        self.last_height = 0

    def __enter__(self):
        if self.is_tty:
            tty.setraw(self.fd)
            sys.stdout.write("\033[?25l")   # hide cursor
            sys.stdout.flush()
        return self

    def __exit__(self, *exc):
        if self.is_tty:
            sys.stdout.write("\033[?25h")   # restore cursor
            sys.stdout.flush()
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved_attrs)

    def render(self, lines):
        """Rewrite the given logical lines in place as one screen region."""
        if not self.is_tty:
            return
        width = shutil.get_terminal_size((80, 24)).columns
        phys = []
        for ln in lines:
            for i in range(0, len(ln), width):
                phys.append(ln[i:i + width])
        out = []
        if self.last_height:
            out.append(f"\033[{self.last_height}A")  # up to region top
        out.append("\r\033[J")                       # column 0, clear below
        for p in phys:
            out.append(p + "\r\n")                   # col-0 after every line
        sys.stdout.write("".join(out))
        sys.stdout.flush()
        self.last_height = len(phys)


def fmt_ee(label, d):
    """One short xyz+rpy line. d is an ee_to_dict() result or None."""
    if d is None:
        return f"{label}   (no pose msg yet)"
    p, r = d["pos"], d["rpy_deg"]
    return (f"{label}  x={p[0]:+.4f} y={p[1]:+.4f} z={p[2]:+.4f}   "
            f"roll={r[0]:+6.1f} pitch={r[1]:+6.1f} yaw={r[2]:+6.1f}")


def main():
    ap = argparse.ArgumentParser(description="Named EE-pose snapshot logger")
    ap.add_argument("--out-dir", default="./ee_logs",
                    help="directory for the snapshot file (default ./ee_logs)")
    ap.add_argument("--snap-file", default="ee_pose_snapshots.jsonl",
                    help="snapshot file name inside --out-dir")
    args = ap.parse_args()

    rclpy.init()
    node = EEPoseLogger()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    os.makedirs(args.out_dir, exist_ok=True)
    snap_path = os.path.join(args.out_dir, args.snap_file)
    fh = open(snap_path, "a", buffering=1)     # line-buffered: survives Ctrl+C

    # The name field is always live: you type straight into it and Enter
    # saves the current pose. The buffer is kept after a save so you can
    # backspace to clear or edit the previous name.
    name_buf = ""
    last_name = None          # name of the most recently saved pose
    n_saved = 0
    last_redraw = 0.0
    dirty = False             # a keystroke was handled -> redraw right now
    last_enter = 0.0          # time of the last Enter acted on; any \r/\n
                              # within ENTER_ECHO_WINDOW of it is an echo

    print("Type a name (backspace to edit);  Enter = save this pose  "
          "Ctrl-C = quit\n"
          f"snapshots -> {snap_path}\n")

    def live_ee_l():
        return ee_to_dict(node.latest["ee_l"])

    def live_ee_r():
        return ee_to_dict(node.latest["ee_r"])

    def save():
        """Capture + write one snapshot: the pose is whatever is newest at the
        Enter press. The buffer is deliberately left intact so the next pose is
        named by backspacing over the previous name."""
        nonlocal last_name, n_saved
        host = time.time()
        name = name_buf.strip() or time.strftime("snap_%H%M%S")
        obj = {
            "name": name,
            "wall_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(host)),
            "saved_at": round(host, 6),
            "ee_l": live_ee_l(),
            "ee_r": live_ee_r(),
            "joints": joints_to_dict(node.latest["joints"]),
        }
        fh.write(json.dumps(obj) + "\n")
        last_name = name
        n_saved += 1

    try:
        with Terminal() as term:
            time.sleep(0.3)     # let the first pose messages arrive
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.01)
                now = time.time()

                if term.is_tty:
                    rd, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if rd:
                        for c in os.read(term.fd, 64).decode(errors="ignore"):
                            if c in ("\r", "\n"):
                                # Raw-mode Enter arrives as \r. A \r/\n within
                                # ENTER_ECHO_WINDOW of the Enter just acted on is
                                # that press's trailing echo -> drop it.
                                if time.time() - last_enter < ENTER_ECHO_WINDOW:
                                    continue
                                last_enter = time.time()
                                save()            # capture pose at THIS Enter press
                                dirty = True
                                continue
                            if c in ("\x7f", "\b"):    # backspace edits the name
                                name_buf = name_buf[:-1]
                                dirty = True
                            elif c == "\x03":          # raw mode hands Ctrl-C over
                                raise KeyboardInterrupt  # as a byte, not a signal
                            elif c.isprintable():
                                name_buf += c            # every printable key is
                                dirty = True             # part of the live name
                            # Esc / stray control bytes are ignored, so an
                            # accidental arrow key can't quit or wipe the name.

                # Redraw the live pose ~20 Hz, and immediately after any
                # keystroke so typing the name feels instant.
                if term.is_tty and (dirty or now - last_redraw >= 0.05):
                    last_redraw = now
                    dirty = False
                    lines = [fmt_ee("L", live_ee_l()),
                             fmt_ee("R", live_ee_r()),
                             f"  name: {name_buf}█",
                             "  last pose name: " + (last_name or "(none yet)")]
                    term.render(lines)
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print(f"\nsaved {n_saved} snapshot(s) -> {snap_path}")


if __name__ == "__main__":
    main()
