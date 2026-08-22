#!/usr/bin/env python3
"""Interactive IK Solver CLI — combined interface for joint management,
EE soft locks, pose save/load, and IK solver control.

Features:
  - List joints with LOCKED/SOFT/FREE status
  - Toggle individual joint soft locks (via /teleop_locks, ±3° slack)
  - Toggle EE yaw/roll/pitch soft locks (submenu, lock both / unlock both)
  - Toggle joint groups (left_arm / right_arm / lift)
  - Save/load poses to file
  - Save/load limit profiles to file, clear live profile
  - Reset to home
  - Show arm group enable/disable status
  - Show kinematic tree

Services used:
  /ik_solver/list_joints        (Trigger)   — list joints with lock status
  /ik_solver/get_tree           (Trigger)   — kinematic tree
  /ik_solver/save_pose          (SaveLoadPose)
  /ik_solver/load_pose          (SaveLoadPose)
  /ik_solver/list_saved_poses   (Trigger)   — memory (unused by CLI, file-only now)
  /ik_solver/toggle_joint_group (ToggleJointGroup)
  /ik_solver/reset_to_home      (Trigger)
  /teleop_locks                 (std_msgs/String) — toggle individual joint locks
  /ik_solver/ee_lock            (std_msgs/String) — toggle EE yaw/roll/pitch locks
  /teleop/limit_profile         (std_msgs/String) — set/clear the EE goal clamp box
"""

import os
import sys
import select
import tty
import termios
import re
import math
import time
import rclpy
from rclpy.node import Node
from ffw_collision_checker.srv import SaveLoadPose, ToggleJointGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# Absolute path to the poses file — must match ffw_ik_solver_teleop.cpp
POSES_FILE = "/home/lys/robotis_ws/src/ai_worker/ffw_collision_checker/config/poses.txt"

# Limit-profile save/load file (CLI-side only; lines are the exact
# `/teleop/limit_profile` `set <arm> ...` messages, so load publishes verbatim).
LIMIT_PROFILE_FILE = "/home/lys/robotis_ws/src/ai_worker/ffw_collision_checker/config/limit_profiles.txt"


# ── Terminal helpers ──────────────────────────────────────────────────

def getch():
    """Read a single character from stdin, handling arrow key escape sequences."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return '\x1b[' + ch3
            return '\x1b'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def clear_screen():
    sys.stdout.write("\033[H\033[J")
    sys.stdout.flush()


def select_menu(options, title="Select an option:", allow_custom=False,
                custom_label="Custom input"):
    """Arrow-key navigable menu with number shortcuts.
    Returns index or None (cancel).
    - Number keys 1-9 → select that option immediately (1-indexed)
    - Left arrow   → cancel/back
    - Right arrow  → confirm (same as Enter)
    - Up/Down arrows → navigate
    - Enter → confirm
    - q / Esc → cancel
    If allow_custom=True and the last option is selected, returns the
    user's typed input string instead of an index.
    """
    if not options:
        return None
    selected = 0
    while True:
        clear_screen()
        sys.stdout.write(f"=== {title} ===\n\n")
        for idx, opt in enumerate(options):
            prefix = f"\033[36m{idx+1}\033[0m " if idx < 9 else "  "
            if idx == selected:
                sys.stdout.write(f"\033[36m➔  {opt}\033[0m    [{idx+1}]\n")
            else:
                sys.stdout.write(f"   {prefix}{opt}\n")
        sys.stdout.write(
            "\n[↑↓=navigate  ←=back  →/Enter=select  1-9=shortcut  q=exit]\n"
        )
        sys.stdout.flush()

        ch = getch()
        if ch == '\x1b[A':       # Up
            selected = (selected - 1) % len(options)
        elif ch == '\x1b[B':     # Down
            selected = (selected + 1) % len(options)
        elif ch == '\x1b[C':     # Right → confirm (same as Enter)
            if allow_custom and selected == len(options) - 1:
                try:
                    sys.stdout.write("\n\033[?25h")
                    sys.stdout.flush()
                    val = input(f"\n{custom_label}: ").strip()
                    return val if val else None
                except (KeyboardInterrupt, EOFError):
                    return None
            return selected
        elif ch == '\x1b[D':     # Left → cancel
            return None
        elif ch in ('\r', '\n'): # Enter → confirm
            if allow_custom and selected == len(options) - 1:
                try:
                    sys.stdout.write("\n\033[?25h")
                    sys.stdout.flush()
                    val = input(f"\n{custom_label}: ").strip()
                    return val if val else None
                except (KeyboardInterrupt, EOFError):
                    return None
            return selected
        elif ch.isdigit() and '1' <= ch <= '9':
            num = int(ch)
            if 1 <= num <= len(options):
                if allow_custom and num == len(options):
                    # Last option with allow_custom → custom input
                    try:
                        sys.stdout.write("\n\033[?25h")
                        sys.stdout.flush()
                        val = input(f"\n{custom_label}: ").strip()
                        return val if val else None
                    except (KeyboardInterrupt, EOFError):
                        return None
                return num - 1
        elif ch.lower() == 'q' or ch == '\x1b' or ch == '\x03':
            return None


def press_enter():
    """Prompt and wait for Enter."""
    try:
        input("\nPress Enter to continue...")
    except (KeyboardInterrupt, EOFError):
        pass


# ── CLI Node ──────────────────────────────────────────────────────────

class IKSolverCLI(Node):
    """Interactive CLI for IK Solver management."""

    def __init__(self):
        super().__init__('ik_solver_cli')
        # ── Clients ────────────────────────────────────────────────
        self.list_joints_client = self.create_client(
            Trigger, '/ik_solver/list_joints')
        self.tree_client = self.create_client(
            Trigger, '/ik_solver/get_tree')
        self.save_client = self.create_client(
            SaveLoadPose, '/ik_solver/save_pose')
        self.load_client = self.create_client(
            SaveLoadPose, '/ik_solver/load_pose')
        self.list_poses_client = self.create_client(
            Trigger, '/ik_solver/list_saved_poses')
        self.toggle_group_client = self.create_client(
            ToggleJointGroup, '/ik_solver/toggle_joint_group')
        self.reset_home_client = self.create_client(
            Trigger, '/ik_solver/reset_to_home')

        # ── Publishers ─────────────────────────────────────────────
        self.lock_pub = self.create_publisher(String, '/teleop_locks', 10)
        self.ee_lock_pub = self.create_publisher(String, '/ik_solver/ee_lock', 10)
        self.limit_profile_pub = self.create_publisher(String, '/teleop/limit_profile', 10)

        # EE lock toggle state: True = locked (both arms), False = unlocked
        self.ee_yaw_locked = False
        self.ee_roll_locked = False
        self.ee_pitch_locked = False

        # Current limit profile: arm ('l'/'r') -> {axis: (lo, hi)}. Populated by
        # create/load so "save limit profile" can snapshot it; {} = none active.
        self._current_profile = {}

        # ── Achieved-pose cache (solver → CLI, world frame "map") ──
        # Latest /ik_solver/achieved_ee_pose_* per arm ('l'/'r'). Populated by
        # the subscription callback whenever rclpy.spin_once runs — the CLI is
        # otherwise blocked in input()/select_menu(), so after every prompt we
        # drain the queue and keep the message with the newest header.stamp.
        self._achieved = {'l': None, 'r': None}
        for arm, topic in (('l', '/ik_solver/achieved_ee_pose_l'),
                           ('r', '/ik_solver/achieved_ee_pose_r')):
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, a=arm: self._achieved.__setitem__(a, msg),
                10)

    # ── Service wrappers ──────────────────────────────────────────

    def wait_for_services(self):
        self.get_logger().info("Connecting to IK Solver services...")
        services = [
            ('/ik_solver/list_joints', self.list_joints_client),
            ('/ik_solver/get_tree', self.tree_client),
            ('/ik_solver/save_pose', self.save_client),
            ('/ik_solver/load_pose', self.load_client),
            ('/ik_solver/list_saved_poses', self.list_poses_client),
            ('/ik_solver/toggle_joint_group', self.toggle_group_client),
            ('/ik_solver/reset_to_home', self.reset_home_client),
        ]
        for name, client in services:
            while not client.wait_for_service(timeout_sec=2.0):
                print(f"Waiting for {name}...")
        print("Connected to all services!")

    # ── Joint listing + parsing ───────────────────────────────────

    def get_joints_raw(self):
        """Call /ik_solver/list_joints and return (raw_message, [(name, status), ...]).
        status is 'LOCKED', 'SOFT', or 'FREE'."""
        req = Trigger.Request()
        future = self.list_joints_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            return "(service unavailable)", []
        msg = res.message

        joints = []
        for line in msg.split('\n'):
            match = re.search(r'\[\d+\]\s*\[(.*?)\]\s*(.*)', line)
            if match:
                raw_status = match.group(1).strip()
                if 'LOCKED' in raw_status:
                    status = 'LOCKED'
                elif 'SOFT' in raw_status:
                    status = 'SOFT'
                else:
                    status = 'FREE'
                jname = match.group(2).strip()
                joints.append((jname, status))
        return msg, joints

    # ── Pose save/load ────────────────────────────────────────────

    def call_save(self, name, to_file):
        req = SaveLoadPose.Request()
        req.pose_name = name
        req.to_file = to_file
        future = self.save_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            return False, "Service call timed out or failed"
        return res.success, res.message

    def call_load(self, name, to_file):
        req = SaveLoadPose.Request()
        req.pose_name = name
        req.to_file = to_file
        future = self.load_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            return False, "Service call timed out or failed"
        return res.success, res.message

    def call_list_memory(self):
        req = Trigger.Request()
        future = self.list_poses_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None or not res.success or not res.message.strip():
            return []
        return [line.strip() for line in res.message.split('\n') if line.strip()]

    # ── Menu actions ──────────────────────────────────────────────

    def action_list_joints(self):
        """Show all joints with LOCKED status."""
        msg, _ = self.get_joints_raw()
        clear_screen()
        print("=== Joint List ===\n")
        print(msg)
        press_enter()

    def action_show_tree(self):
        """Show kinematic tree."""
        req = Trigger.Request()
        future = self.tree_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        clear_screen()
        print("=== Kinematic Tree ===\n")
        print(res.message if res else "(service unavailable)")
        press_enter()

    def action_toggle_joint(self):
        """Toggle lock on individual joints via /teleop_locks. Arrow-key menu with live status."""
        while True:
            _, joints = self.get_joints_raw()
            if not joints:
                print("No joints available.")
                press_enter()
                return

            options = []
            for jname, status in joints:
                color = "\033[31m" if status == 'LOCKED' else ("\033[33m" if status == 'SOFT' else "\033[32m")
                options.append(f"{color}{status}\033[0m  {jname}")

            options.append("\033[90m← Back\033[0m")

            idx = select_menu(options, "Toggle Joint Lock  (soft, ±3° slack)  [1-9=shortcut  ←=back]")
            if idx is None or idx == len(options) - 1:
                return

            jname = joints[idx][0]
            # Toggle it — publish to /teleop_locks
            msg = String()
            msg.data = jname
            self.lock_pub.publish(msg)
            # Let the node process
            rclpy.spin_once(self, timeout_sec=0.1)
            # Show updated state immediately
            clear_screen()
            print(f"Toggled: {jname}\n")

    def action_toggle_group(self):
        """Toggle enable/disable of arm groups. Arrow-key menu with live status."""
        while True:
            _, joints = self.get_joints_raw()

            arm_l_joints = [n for n, s in joints if n.startswith('arm_l')]
            arm_r_joints = [n for n, s in joints if n.startswith('arm_r')]
            lift_joints  = [n for n, s in joints if n.startswith('lift')]

            def fmt_group(group_joints, label):
                total = len(group_joints)
                if total == 0:
                    return f"\033[90m?\033[0m  {label}"
                locked = sum(1 for n, s in joints if n in group_joints and s == 'LOCKED')
                if locked == total:
                    return f"\033[31mDISABLED\033[0m  {label}  ({total}/{total} locked)"
                elif locked == 0:
                    return f"\033[32mENABLED\033[0m   {label}  (0/{total} locked)"
                else:
                    return f"\033[33mPARTIAL\033[0m  {label}  ({locked}/{total} locked)"

            options = [
                fmt_group(arm_l_joints, "Left arm"),
                fmt_group(arm_r_joints, "Right arm"),
                fmt_group(lift_joints,  "Lift"),
                "\033[90m← Back\033[0m",
            ]

            idx = select_menu(options, "Toggle Joint Group  [1-3=toggle  Enter=toggle  ←=back]")
            if idx is None or idx >= 3:
                return

            group_map = {0: "left_arm", 1: "right_arm", 2: "lift"}
            group_name = group_map[idx]

            req = ToggleJointGroup.Request()
            req.group_name = group_name
            future = self.toggle_group_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            clear_screen()
            if res is None:
                print(f"Toggled {group_name}: FAILED — service timed out")
            else:
                print(f"Toggled {group_name}: {'SUCCESS' if res.success else 'FAILED'}")
                print(res.message)
            print()

    def action_show_status(self):
        """Show which arm groups are enabled/disabled."""
        _, joints = self.get_joints_raw()

        # Determine group status: a group is "disabled" if ALL its joints are LOCKED
        arm_l_joints = [n for n, s in joints if n.startswith('arm_l')]
        arm_r_joints = [n for n, s in joints if n.startswith('arm_r')]
        lift_joints  = [n for n, s in joints if n.startswith('lift')]

        def group_status(group_joints, label):
            locked = sum(1 for n, s in joints if n in group_joints and s == 'LOCKED')
            total = len(group_joints)
            if total == 0:
                return f"{label}: \033[90munknown\033[0m"
            if locked == total:
                return f"{label}: \033[31mDISABLED\033[0m (all {total} joints LOCKED)"
            elif locked == 0:
                return f"{label}: \033[32mENABLED\033[0m (all {total} joints free)"
            else:
                return f"{label}: \033[33mPARTIAL\033[0m ({locked}/{total} joints LOCKED)"

        clear_screen()
        print("=== Arm Group Status ===\n")
        print(group_status(lift_joints,  "Lift"))
        print(group_status(arm_l_joints, "Left arm"))
        print(group_status(arm_r_joints, "Right arm"))
        print()
        print("(Group status is derived from individual joint lock state.)")
        press_enter()

    def _set_ee_lock_state(self, axis, locked):
        """Helper: publish lock or unlock commands for an axis (yaw/roll/pitch)
        to BOTH arms."""
        action = 'lock' if locked else 'unlock'
        for arm in ['l', 'r']:
            msg = String()
            msg.data = f"{action} {axis}_{arm}"
            self.ee_lock_pub.publish(msg)

    def action_toggle_ee_rpy(self):
        """EE RPY lock submenu: toggle yaw/roll/pitch individually. Loop until
        ← Back so several axes can be toggled without returning to the main
        menu."""
        while True:
            yaw_color = "\033[31m" if self.ee_yaw_locked else "\033[32m"
            roll_color = "\033[31m" if self.ee_roll_locked else "\033[32m"
            pitch_color = "\033[31m" if self.ee_pitch_locked else "\033[32m"
            yaw_label = "LOCKED" if self.ee_yaw_locked else "unlocked"
            roll_label = "LOCKED" if self.ee_roll_locked else "unlocked"
            pitch_label = "LOCKED" if self.ee_pitch_locked else "unlocked"
            options = [
                f"Toggle EE yaw lock   [{yaw_color}{yaw_label}\033[0m]",
                f"Toggle EE roll lock  [{roll_color}{roll_label}\033[0m]",
                f"Toggle EE pitch lock [{pitch_color}{pitch_label}\033[0m]",
                "\033[90m← Back\033[0m",
            ]
            idx = select_menu(options, "Toggle EE RPY Lock  (lock both / unlock both)")
            if idx is None or idx == len(options) - 1:
                return
            [self.action_toggle_ee_yaw,
             self.action_toggle_ee_roll,
             self.action_toggle_ee_pitch][idx]()

    def action_toggle_ee_yaw(self):
        """Toggle EE yaw lock: lock both / unlock both."""
        self.ee_yaw_locked = not self.ee_yaw_locked
        self._set_ee_lock_state('yaw', self.ee_yaw_locked)
        label = 'LOCKED' if self.ee_yaw_locked else 'UNLOCKED'
        clear_screen()
        print(f"EE yaw lock: {label}  (both arms)\n")

    def action_toggle_ee_roll(self):
        """Toggle EE roll lock: lock both / unlock both."""
        self.ee_roll_locked = not self.ee_roll_locked
        self._set_ee_lock_state('roll', self.ee_roll_locked)
        label = 'LOCKED' if self.ee_roll_locked else 'UNLOCKED'
        clear_screen()
        print(f"EE roll lock: {label}  (both arms)\n")

    def action_toggle_ee_pitch(self):
        """Toggle EE pitch lock: lock both / unlock both."""
        self.ee_pitch_locked = not self.ee_pitch_locked
        self._set_ee_lock_state('pitch', self.ee_pitch_locked)
        label = 'LOCKED' if self.ee_pitch_locked else 'UNLOCKED'
        clear_screen()
        print(f"EE pitch lock: {label}  (both arms)\n")

    # ── Limit-profile helpers ──────────────────────────────────────

    @staticmethod
    def _quat_to_rpy(q):
        """Extrinsic XYZ Euler RPY from a quaternion — mirrors joy_hand
        extract_rpy(): pitch=asin(R02), yaw=atan2(-R01,R00), roll=atan2(-R12,R22).
        Returns (roll, pitch, yaw) in radians."""
        x, y, z, w = q.x, q.y, q.z, q.w
        # Rotation matrix from quaternion
        R00 = 1.0 - 2.0 * (y * y + z * z)
        R01 = 2.0 * (x * y - z * w)
        R02 = 2.0 * (x * z + y * w)
        R12 = 2.0 * (y * z - x * w)
        R22 = 1.0 - 2.0 * (x * x + y * y)
        sin_pitch = R02
        if abs(sin_pitch) >= 0.9999999:
            pitch = math.copysign(math.pi / 2.0, sin_pitch)
            yaw = 0.0
        else:
            pitch = math.asin(sin_pitch)
            yaw = math.atan2(-R01, R00)
        roll = math.atan2(-R12, R22)
        return (roll, pitch, yaw)

    @staticmethod
    def _stamp_ns(pose):
        """header.stamp as a float in seconds."""
        s = pose.header.stamp
        return s.sec + s.nanosec * 1e-9

    def _spin_drain(self, n=8):
        """Run rclpy.spin_once n times so queued subscriptions are processed.
        Called after input()/select_menu() blocks the thread."""
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_achieved(self, arm, timeout=10.0):
        """Spin until a fresh (≤1 s) achieved pose exists for this arm.
        The CLI otherwise only spins during service calls; the profile flow
        needs live poses while prompting."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin_drain(2)
            pose = self._achieved.get(arm)
            if pose is not None:
                age = time.time() - self._stamp_ns(pose)
                if age <= 1.0:
                    return True
        return False

    def _current(self, arm):
        """(pos, rpy) of the latest achieved pose for this arm, or None."""
        pose = self._achieved.get(arm)
        if pose is None:
            return None
        p = pose.pose.position
        return (p.x, p.y, p.z), self._quat_to_rpy(pose.pose.orientation)

    def _capture_pose(self, arm, axis, label):
        """Live-position capture for limit-profile registration.

        Prints the registered axis's value continuously while the operator
        moves the arm, overwriting the same line with \r (the old in-place
        trick) so the terminal never scrolls. input() would block the thread
        and freeze the display, so we spin in a raw-mode loop: achieved poses
        keep arriving and the value refreshes live; Enter captures the NEWEST
        achieved pose (drain after, so the capture isn't mid-motion)."""
        is_angular = axis in ('roll', 'pitch', 'yaw')
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)  # no echo / no line-buffering while displaying
            sys.stdout.write(
                f"\n    {axis.upper()} — move arm to the {label} pose, "
                f"press Enter to capture\n")
            sys.stdout.flush()
            while True:
                rclpy.spin_once(self, timeout_sec=0.02)
                pose = self._achieved.get(arm)
                if pose is not None:
                    p = pose.pose.position
                    rpy = self._quat_to_rpy(pose.pose.orientation)
                    v = self._axis_value(axis, (p.x, p.y, p.z), rpy)
                    if is_angular:
                        line = f"\r\033[2K    {axis.upper()}: {math.degrees(v):8.2f}°"
                    else:
                        line = f"\r\033[2K    {axis.upper()}: {v*100:8.2f} cm"
                    sys.stdout.write(line)
                    sys.stdout.flush()
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r:
                    ch = os.read(fd, 1)
                    if ch in (b'\r', b'\n'):
                        break
                    if ch == b'\x03':  # Ctrl-C → cancel
                        raise KeyboardInterrupt
        except (KeyboardInterrupt, EOFError):
            print("\r\033[2K  Cancelled.")
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            sys.stdout.write("\r\033[2K")  # clear the live line
            sys.stdout.flush()

        # Drain the queue and keep the pose with the newest header.stamp —
        # the arm may still be settling as Enter lands.
        best = None
        for _ in range(12):
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self._achieved.get(arm)
            if pose is not None:
                if best is None or self._stamp_ns(pose) > self._stamp_ns(best):
                    best = pose
        if best is None:
            print(f"  (no achieved pose received for {label} arm — aborting this arm)")
            return None
        p = best.pose.position
        return (p.x, p.y, p.z), self._quat_to_rpy(best.pose.orientation)

    @staticmethod
    def _centered_bounds(center, delta):
        """Linear [lo, hi] box = center ± delta, inside (−π, π] for angular
        axes. Picks the center representative that keeps the box off the ±π
        seam so the mapper's linear clamp stays valid (179° ± 10° becomes
        [169°, 180°)∪(−180°, −171°] represented as [−179°, −171°], NOT the
        broken [169°, 189°])."""
        c = (center + math.pi) % (2.0 * math.pi) - math.pi  # into (−π, π]
        if c + delta > math.pi:
            c -= 2.0 * math.pi
        elif c - delta < -math.pi:
            c += 2.0 * math.pi
        return c - delta, c + delta

    @staticmethod
    def _angular_interval(a, b):
        """Linear [lo, hi] box in (−π, π] representing the SHORT circular arc
        between two registered angles a, b. Naive min/max across the seam would
        produce a box spanning the wrong (long) arc — e.g. 3.1 and −3.0 are
        0.18 rad apart through π, not 6.1 rad."""
        d = (b - a + math.pi) % (2.0 * math.pi) - math.pi  # short signed dist
        if abs(d) > math.pi - 1e-9:
            d = math.pi if d >= 0 else -math.pi  # exactly opposite: pick a side
        lo, hi = a, a + d
        if hi > math.pi:
            lo -= 2.0 * math.pi
            hi -= 2.0 * math.pi
        elif lo < -math.pi:
            lo += 2.0 * math.pi
            hi += 2.0 * math.pi
        return min(lo, hi), max(lo, hi)

    def _ask_yes_no(self, prompt):
        try:
            ans = input(prompt + " (y/n) > ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            return False
        return ans in ('y', 'yes')

    def _axis_value(self, axis, pos, rpy):
        """Current value of an axis: 'x'/'y'/'z' from pos, rpy axes by name."""
        if axis == 'x':
            return pos[0]
        if axis == 'y':
            return pos[1]
        if axis == 'z':
            return pos[2]
        return rpy[['roll', 'pitch', 'yaw'].index(axis)]

    def _skip_lock_value(self, axis, arm, is_angular):
        """Live skip decision for one axis. Shows the axis value updating in
        place (same \r overwrite trick as capture) while the operator moves the
        arm; 'y' locks in the value read at that moment as the skip center.
        Returns (True, center) to skip, (False, None) to register, or
        (None, None) to cancel."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        center = None
        try:
            tty.setraw(fd)  # no echo / no line-buffering while displaying
            sys.stdout.write(f"\n    {axis.upper()} — y=skip±5 / n=register > ")
            sys.stdout.flush()
            while True:
                rclpy.spin_once(self, timeout_sec=0.02)
                pose = self._achieved.get(arm)
                if pose is not None:
                    p = pose.pose.position
                    rpy = self._quat_to_rpy(pose.pose.orientation)
                    center = self._axis_value(axis, (p.x, p.y, p.z), rpy)
                    if is_angular:
                        line = (f"\r\033[2K    {axis.upper()}: "
                                f"{math.degrees(center):8.2f}° — "
                                f"y=skip±5 / n=register > ")
                    else:
                        line = (f"\r\033[2K    {axis.upper()}: "
                                f"{center * 100:8.2f} cm — "
                                f"y=skip±5 / n=register > ")
                    sys.stdout.write(line)
                    sys.stdout.flush()
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r:
                    ch = os.read(fd, 1)
                    if ch in (b'y', b'Y'):
                        break  # lock in the value just shown
                    if ch in (b'n', b'N', b'\r', b'\n'):
                        return False, None  # go to register
                    if ch == b'\x03':  # Ctrl-C → cancel
                        return None, None
        except (KeyboardInterrupt, EOFError):
            return None, None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            sys.stdout.write("\r\033[2K")  # clear the live line
            sys.stdout.flush()
        if center is None:
            print("    (no achieved pose received — aborting this axis)")
            return None, None
        return True, center

    def _axis_bounds(self, axis, delta, is_angular):
        """Ask skip/register for one axis and return (lo, hi). Skip shows the
        live value updating in place and locks in whatever it reads when 'y' is
        pressed (center ± delta); register captures two physical poses."""
        skip, center = self._skip_lock_value(axis, self._profile_arm, is_angular)
        if skip is None:
            return None  # cancelled
        if skip:
            if is_angular:
                return self._centered_bounds(center, delta)
            return center - 0.05, center + 0.05

        print(f"    Register {axis}: move the arm to 2 poses (Enter after each).")
        p1 = self._capture_pose(self._profile_arm, axis, "first")
        if p1 is None:
            return None
        p2 = self._capture_pose(self._profile_arm, axis, "second")
        if p2 is None:
            return None
        v1 = self._axis_value(axis, *p1)
        v2 = self._axis_value(axis, *p2)

        if is_angular:
            if abs(v2 - v1) > math.pi:
                print(f"    (warn: registered {axis} poses are "
                      f"{math.degrees(abs(v2 - v1)):.0f}° apart linearly — "
                      f"using the short arc between them)")
            lo, hi = self._angular_interval(v1, v2)
        else:
            lo, hi = min(v1, v2), max(v1, v2)

        unit = 'deg' if is_angular else 'cm'
        if abs(hi - lo) < 1e-6:
            # Both registered poses agree on this axis: a zero-width box would
            # silently freeze it. Widen to the registered value ± skip delta.
            print(f"    (registered poses agree on {axis}; widening to "
                  f"±5 {unit})")
            if is_angular:
                return self._centered_bounds(v1, delta)
            return v1 - 0.05, v1 + 0.05
        return lo, hi

    # ── Limit-profile actions ──────────────────────────────────────

    def action_create_limit_profile(self):
        """Interactive per-arm position+orientation box, clamped by the mapper
        onto the absolute world-frame goal. Per-arm skip → ±5cm/±10° around
        current; unskipped arms cycle yaw→roll→pitch→z→y→x, skip or register."""
        clear_screen()
        print("=== Create Manual Limit Profile ===\n")
        print("The mapper clamps the EE goal inside these bounds (world frame "
              "\"map\"). Angles are extrinsic XYZ RPY.\n")

        profile = {}   # arm -> {'x':..,'y':..,'z':..,'roll':..,'pitch':..,'yaw':..}
        for arm, arm_name in (('l', 'left'), ('r', 'right')):
            self._profile_arm = arm
            if not self._wait_achieved(arm):
                print(f"\n{arm_name} arm: no fresh achieved pose — skipped.")
                continue
            pos, rpy = self._current(arm)
            print(f"\n{arm_name.upper()} arm current: "
                  f"pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m "
                  f"rpy=({math.degrees(rpy[0]):.1f}, {math.degrees(rpy[1]):.1f}, "
                  f"{math.degrees(rpy[2]):.1f}) deg")

            if self._ask_yes_no(f"Skip entire {arm_name} arm? "
                                f"(±5 cm / ±10° around current)"):
                profile[arm] = {
                    'x': (pos[0] - 0.05, pos[0] + 0.05),
                    'y': (pos[1] - 0.05, pos[1] + 0.05),
                    'z': (pos[2] - 0.05, pos[2] + 0.05),
                    'roll': self._centered_bounds(rpy[0], math.radians(10)),
                    'pitch': (rpy[1] - math.radians(10), rpy[1] + math.radians(10)),
                    'yaw': self._centered_bounds(rpy[2], math.radians(10)),
                }
                continue

            arm_profile = {}
            for axis in ('yaw', 'roll', 'pitch', 'z', 'y', 'x'):
                is_angular = axis in ('yaw', 'roll', 'pitch')
                delta = math.radians(5) if is_angular else 0.05
                bounds = self._axis_bounds(axis, delta, is_angular)
                if bounds is None:
                    print(f"    {axis}: aborted — arm profile dropped.")
                    arm_profile = None
                    break
                arm_profile[axis] = bounds
            if arm_profile:
                profile[arm] = arm_profile

        if not profile:
            print("\nNo profiles created.")
            return

        self._current_profile = profile
        self._publish_limit_profile(profile)

        # Summary table in the mapper's units (deg/cm), flagging any axis whose
        # current pose sits outside the box.
        print("\n=== Limit Profile Set ===")
        self._print_profile_summary(profile)
        print("\n(Clamping is applied in the mapper; drive past a bound to "
              "verify.)")
        press_enter()

    def _publish_limit_profile(self, profile):
        """Publish a {arm: {axis: (lo, hi)}} profile as `set <arm> ...` messages
        on /teleop/limit_profile. Axis order matches the mapper's parser:
        px py pz roll pitch yaw."""
        order = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
        for arm, bounds in profile.items():
            flat = [v for axis in order for v in bounds[axis]]
            msg = String()
            msg.data = 'set {} {}'.format(arm, ' '.join(f'{v:.6f}' for v in flat))
            self.limit_profile_pub.publish(msg)
            self._spin_drain(1)

    def _print_profile_summary(self, profile):
        """Per-arm bound table in the mapper's units (deg/cm), flagging any axis
        whose current pose sits outside the box."""
        for arm, bounds in profile.items():
            arm_name = 'left' if arm == 'l' else 'right'
            print(f"\n{arm_name.upper()} arm:")
            cur = self._current(arm)
            for axis, (lo, hi) in bounds.items():
                is_angular = axis in ('yaw', 'roll', 'pitch')
                if is_angular:
                    print(f"  {axis:5s} [{math.degrees(lo):7.1f}°, "
                          f"{math.degrees(hi):7.1f}°]")
                else:
                    print(f"  {axis:5s} [{lo*100:7.1f} cm, {hi*100:7.1f} cm]")
                if cur is not None:
                    v = self._axis_value(axis, *cur)
                    if v < lo or v > hi:
                        print(f"         ^ current {axis} is OUTSIDE the box!")

    def action_clear_limit_profile(self):
        """Drop the limit profile for both arms (restore full range)."""
        for arm in ('l', 'r'):
            msg = String()
            msg.data = f'clear {arm}'
            self.limit_profile_pub.publish(msg)
        self._spin_drain(1)
        self._current_profile = {}
        clear_screen()
        print("Limit profile cleared for both arms — full range restored.\n")
        press_enter()

    # ── Limit-profile file save/load ───────────────────────────────

    @staticmethod
    def _read_limit_profiles():
        """Parse LIMIT_PROFILE_FILE into {name: [profile_line, ...]}. Lines are
        the exact `set <arm> <12 bounds>` messages, so they can be published
        verbatim on load. Sections start with [name]."""
        if not os.path.exists(LIMIT_PROFILE_FILE):
            return {}
        profiles = {}
        current = None
        with open(LIMIT_PROFILE_FILE, 'r') as f:
            for line in f:
                line = line.rstrip('\n')
                if line.startswith('[') and line.endswith(']'):
                    current = line[1:-1]
                    profiles.setdefault(current, [])
                elif current is not None and line.startswith('set '):
                    profiles[current].append(line)
        return profiles

    def _profile_from_lines(self, lines):
        """Parse `set <arm> <12 bounds>` lines back into a
        {arm: {axis: (lo, hi)}} profile for the summary table."""
        order = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
        profile = {}
        for line in lines:
            parts = line.split()
            if len(parts) != 14:  # set + arm + 12 bounds
                continue
            arm = parts[1]
            try:
                vals = [float(v) for v in parts[2:]]
            except ValueError:
                continue
            profile[arm] = {axis: (vals[i * 2], vals[i * 2 + 1])
                            for i, axis in enumerate(order)}
        return profile

    def action_save_limit_profile(self):
        """Snapshot the current live limit profile to file under a name."""
        clear_screen()
        print("=== Save current limit profile to file ===")
        if not self._current_profile:
            print("\nNo limit profile is active. Create or load one first.")
            press_enter()
            return
        try:
            name = input("Enter profile name: ").strip()
            if not name:
                print("Cancelled: Name cannot be empty.")
                press_enter()
                return
            if name.startswith('[') or ']' in name:
                print("Cancelled: name cannot contain '[' or ']'.")
                press_enter()
                return
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
            press_enter()
            return

        lines = [f"[{name}]"]
        order = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
        for arm, bounds in self._current_profile.items():
            flat = [v for axis in order for v in bounds[axis]]
            lines.append('set {} {}'.format(arm, ' '.join(f'{v:.6f}' for v in flat)))

        # Replace any existing section with the same name.
        existing = self._read_limit_profiles()
        sections = {n: l for n, l in existing.items() if n != name}
        out_lines = []
        for n, section_lines in sections.items():
            out_lines.append(f"[{n}]")
            out_lines.extend(section_lines)
        out_lines.extend(lines)

        with open(LIMIT_PROFILE_FILE, 'w') as f:
            f.write('\n'.join(out_lines) + '\n')
        print(f"\nSaved profile '{name}' to {LIMIT_PROFILE_FILE}")
        press_enter()

    def action_load_limit_profile(self):
        """Load a named limit profile from file and publish it (fast restore)."""
        clear_screen()
        print("=== Load limit profile from file ===")
        profiles = self._read_limit_profiles()
        if not profiles:
            print(f"\nNo saved limit profiles found ({LIMIT_PROFILE_FILE} "
                  f"does not exist or is empty).")
            press_enter()
            return
        names = list(profiles.keys())
        selected = select_menu(names, "Select a limit profile to load")
        if selected is None:
            return
        name = names[selected]
        lines = profiles[name]

        profile = self._profile_from_lines(lines)
        if not profile:
            print(f"\nProfile '{name}' has no parseable set lines.")
            press_enter()
            return
        self._current_profile = profile
        self._publish_limit_profile(profile)
        clear_screen()
        print(f"=== Limit Profile Loaded: '{name}' ===")
        self._print_profile_summary(profile)
        press_enter()

    def action_save_pose(self, to_file):
        """Save pose to memory or file."""
        label = "file" if to_file else "memory"
        clear_screen()
        print(f"=== Save current pose to {label} ===")
        try:
            name = input(f"Enter pose name for {label}: ").strip()
            if not name:
                print("Cancelled: Name cannot be empty.")
                press_enter()
                return
            success, msg = self.call_save(name, to_file)
            print(f"\nResult: {'SUCCESS' if success else 'FAILED'}\n{msg}")
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
        press_enter()

    def action_load_pose(self, from_file):
        """Load a pose from memory or file."""
        label = "file" if from_file else "memory"

        if from_file:
            # List poses from poses.txt
            filepath = POSES_FILE
            if not os.path.exists(filepath):
                print(f"\nNo saved poses found ({filepath} does not exist).")
                press_enter()
                return
            poses = []
            with open(filepath, 'r') as f:
                for line in f:
                    if line.startswith('[') and line.endswith(']\n'):
                        poses.append(line[1:-2])
            if not poses:
                print("\nNo saved poses found in poses.txt.")
                press_enter()
                return
        else:
            # List poses from memory
            poses = self.call_list_memory()
            if not poses:
                print("\nNo poses found in memory.")
                press_enter()
                return

        selected = select_menu(poses, f"Select a pose to load from {label}")
        if selected is None:
            return

        target_pose = poses[selected]
        success, msg = self.call_load(target_pose, from_file)
        print(f"\nResult: {'SUCCESS' if success else 'FAILED'}\n{msg}")
        press_enter()

    def action_reset_home(self):
        """Reset all joints to home configuration."""
        clear_screen()
        print("=== Reset to Home Configuration ===")
        try:
            confirm = input(
                "This will re-enable all groups and move all joints "
                "to home.\nConfirm? (y/n) > "
            ).strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
            return

        if confirm != 'y':
            print("Cancelled.")
            press_enter()
            return

        req = Trigger.Request()
        future = self.reset_home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            print("\nResult: FAILED — service call timed out")
        else:
            print(f"\nResult: {'SUCCESS' if res.success else 'FAILED'}")
            print(res.message)
        press_enter()

    # ── Main loop ──────────────────────────────────────────────────

    def run(self):
        self.wait_for_services()

        while True:
            # Build menu with live EE lock status colors
            yaw_color = "\033[31m" if self.ee_yaw_locked else "\033[32m"
            roll_color = "\033[31m" if self.ee_roll_locked else "\033[32m"
            pitch_color = "\033[31m" if self.ee_pitch_locked else "\033[32m"
            yaw_label = "LOCKED" if self.ee_yaw_locked else "unlocked"
            roll_label = "LOCKED" if self.ee_roll_locked else "unlocked"
            pitch_label = "LOCKED" if self.ee_pitch_locked else "unlocked"
            main_options = [
                "List joints                         (LOCKED/SOFT/FREE status)",
                "Toggle joint lock                   (soft, with slack)",
                f"Toggle EE RPY lock     [yaw:{yaw_color}{yaw_label}\033[0m "
                f"roll:{roll_color}{roll_label}\033[0m "
                f"pitch:{pitch_color}{pitch_label}\033[0m]",
                "Toggle arm group                    (left_arm / right_arm / lift)",
                "Save pose to file                   (persistent)",
                "Load pose from file                 (from poses.txt)",
                "Save limit profile                  (current live box to file)",
                "Load limit profile                  (named box from file)",
                "Clear limit profile                 (both arms, restore range)",
                "Create limit profile                 (manual, per-arm box)",
                "Reset to home                       (re-enable all, go home)",
                "Show arm group status               (which groups enabled)",
                "Show kinematic tree",
                "Exit",
            ]

            idx = select_menu(main_options, "IK Solver CLI")
            if idx is None or idx == len(main_options) - 1:
                clear_screen()
                print("Exiting. Goodbye!")
                break

            action_map = {
                0:  self.action_list_joints,
                1:  self.action_toggle_joint,
                2:  self.action_toggle_ee_rpy,
                3:  self.action_toggle_group,
                4:  lambda: self.action_save_pose(to_file=True),
                5:  lambda: self.action_load_pose(from_file=True),
                6:  self.action_save_limit_profile,
                7:  self.action_load_limit_profile,
                8:  self.action_clear_limit_profile,
                9:  self.action_create_limit_profile,
                10: self.action_reset_home,
                11: self.action_show_status,
                12: self.action_show_tree,
            }

            action = action_map.get(idx)
            if action:
                action()


def main(args=None):
    rclpy.init(args=args)
    cli = IKSolverCLI()
    try:
        cli.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        try:
            cli.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
