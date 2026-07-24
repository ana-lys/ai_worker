#!/usr/bin/env python3
"""Interactive IK Solver CLI — combined interface for joint management,
EE soft locks, pose save/load, and IK solver control.

Features:
  - List joints with LOCKED/SOFT/FREE status
  - Toggle individual joint soft locks (via /teleop_locks, ±3° slack)
  - Toggle EE yaw/roll soft locks (lock both / unlock both)
  - Toggle joint groups (left_arm / right_arm / lift)
  - Save/load poses to file
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
  /ik_solver/ee_lock            (std_msgs/String) — toggle EE yaw/roll locks
"""

import os
import sys
import tty
import termios
import re
import rclpy
from rclpy.node import Node
from ffw_collision_checker.srv import SaveLoadPose, ToggleJointGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String

# Absolute path to the poses file — must match ffw_ik_solver_teleop.cpp
POSES_FILE = "/home/lys/robotis_ws/src/ai_worker/ffw_collision_checker/config/poses.txt"


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

        # EE lock toggle state: True = locked (both arms), False = unlocked
        self.ee_yaw_locked = False
        self.ee_roll_locked = False

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
        """Helper: publish lock or unlock commands for an axis (yaw/roll) to BOTH arms."""
        action = 'lock' if locked else 'unlock'
        for arm in ['l', 'r']:
            msg = String()
            msg.data = f"{action} {axis}_{arm}"
            self.ee_lock_pub.publish(msg)

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

    def action_list_poses(self):
        """List all saved poses from file."""
        clear_screen()
        print("=== Saved Poses (file) ===\n")
        filepath = POSES_FILE
        if os.path.exists(filepath):
            poses = []
            with open(filepath, 'r') as f:
                for line in f:
                    if line.startswith('[') and line.endswith(']\n'):
                        poses.append(line[1:-2])
            if poses:
                for p in poses:
                    print(f"  - {p}")
            else:
                print("  (none)")
        else:
            print("  (no poses.txt file)")
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
            yaw_label = "LOCKED" if self.ee_yaw_locked else "unlocked"
            roll_label = "LOCKED" if self.ee_roll_locked else "unlocked"
            main_options = [
                "List joints                         (LOCKED/SOFT/FREE status)",
                "Toggle joint lock                   (soft, with slack)",
                f"Toggle EE yaw lock   [{yaw_color}{yaw_label}\033[0m]",
                f"Toggle EE roll lock  [{roll_color}{roll_label}\033[0m]",
                "Toggle arm group                    (left_arm / right_arm / lift)",
                "Save pose to file                   (persistent)",
                "Load pose from file                 (from poses.txt)",
                "List saved poses                    (file only)",
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
                2:  self.action_toggle_ee_yaw,
                3:  self.action_toggle_ee_roll,
                4:  self.action_toggle_group,
                5:  lambda: self.action_save_pose(to_file=True),
                6:  lambda: self.action_load_pose(from_file=True),
                7:  self.action_list_poses,
                8:  self.action_reset_home,
                9:  self.action_show_status,
                10: self.action_show_tree,
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
