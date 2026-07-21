#!/usr/bin/env python3
"""Interactive IK Solver CLI — combined interface for joint management,
pose save/load, and IK solver control.

Merges the old ik_cli.py + pose_cli.py and adds missing features:
  - Toggle joint groups (left_arm / right_arm / lift)
  - Reset to home
  - Show current arm group enable/disable status

Services used:
  /ik_solver/list_joints        (Trigger)   — list joints with lock status
  /ik_solver/get_tree           (Trigger)   — kinematic tree
  /ik_solver/save_pose          (SaveLoadPose)
  /ik_solver/load_pose          (SaveLoadPose)
  /ik_solver/list_saved_poses   (Trigger)
  /ik_solver/toggle_joint_group (ToggleJointGroup)
  /ik_solver/reset_to_home      (Trigger)
  /teleop_locks                 (std_msgs/String) — toggle individual joint locks
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
    """Arrow-key navigable menu. Returns index or None (cancel).
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
            if idx == selected:
                sys.stdout.write(f"\033[36m➔  {opt}\033[0m\n")
            else:
                sys.stdout.write(f"   {opt}\n")
        sys.stdout.write(
            "\nUse [Up/Down] arrows to move, [Enter] to select, "
            "[q]/[Esc]/[Ctrl+C] to return/exit.\n"
        )
        sys.stdout.flush()

        ch = getch()
        if ch == '\x1b[A':
            selected = (selected - 1) % len(options)
        elif ch == '\x1b[B':
            selected = (selected + 1) % len(options)
        elif ch in ('\r', '\n'):
            if allow_custom and selected == len(options) - 1:
                try:
                    sys.stdout.write("\n\033[?25h")  # show cursor
                    sys.stdout.flush()
                    val = input(f"\n{custom_label}: ").strip()
                    return val if val else None
                except (KeyboardInterrupt, EOFError):
                    return None
            return selected
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
        """Call /ik_solver/list_joints and return (raw_message, [(name, is_locked), ...])."""
        req = Trigger.Request()
        future = self.list_joints_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result().message

        joints = []
        for line in msg.split('\n'):
            match = re.search(r'\[\d+\]\s*\[(.*?)\]\s*(.*)', line)
            if match:
                is_locked = 'LOCKED' in match.group(1)
                jname = match.group(2).strip()
                joints.append((jname, is_locked))
        return msg, joints

    # ── Pose save/load ────────────────────────────────────────────

    def call_save(self, name, to_file):
        req = SaveLoadPose.Request()
        req.pose_name = name
        req.to_file = to_file
        future = self.save_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res.success, res.message

    def call_load(self, name, to_file):
        req = SaveLoadPose.Request()
        req.pose_name = name
        req.to_file = to_file
        future = self.load_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res.success, res.message

    def call_list_memory(self):
        req = Trigger.Request()
        future = self.list_poses_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res.success or not res.message.strip():
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
        clear_screen()
        print("=== Kinematic Tree ===\n")
        print(future.result().message)
        press_enter()

    def action_toggle_joint(self):
        """Toggle lock on individual joints via /teleop_locks."""
        _, joints = self.get_joints_raw()
        clear_screen()
        print("Current joints:")
        for jname, locked in joints:
            status = "\033[31mLOCKED\033[0m" if locked else "\033[32mFREE\033[0m"
            print(f"  {status}  {jname}")

        print("\n--- Toggle Joint Lock ---")
        try:
            raw = input(
                "\nEnter joint names or prefixes (space/comma separated, "
                "e.g. 'arm_l' or 'arm_l_joint3'): "
            ).strip()
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
            press_enter()
            return
        if not raw:
            return

        tokens = [t.strip() for t in re.split(r'[,\s]+', raw) if t.strip()]
        _, all_joints = self.get_joints_raw()
        matched = []
        for t in tokens:
            for jname, _ in all_joints:
                if jname.startswith(t) or t == jname:
                    if jname not in matched:
                        matched.append(jname)

        if not matched:
            print("No joints matched your input.")
            press_enter()
            return

        print("\nThe following joints will be toggled:")
        for j in matched:
            print(f"  - {j}")

        try:
            confirm = input("\nConfirm? (y/n) > ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
            press_enter()
            return

        if confirm == 'y':
            for jname in matched:
                msg = String()
                msg.data = jname
                self.lock_pub.publish(msg)
            print("Sent toggle requests!")
        else:
            print("Cancelled.")
        press_enter()

    def action_toggle_group(self):
        """Toggle enable/disable of arm groups."""
        options = [
            "left_arm  — toggle left arm on/off",
            "right_arm — toggle right arm on/off",
            "lift      — toggle lift on/off",
        ]
        idx = select_menu(options, "Toggle Joint Group")
        if idx is None:
            return

        group_map = {0: "left_arm", 1: "right_arm", 2: "lift"}
        group_name = group_map[idx]

        req = ToggleJointGroup.Request()
        req.group_name = group_name
        future = self.toggle_group_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        print(f"\nResult: {'SUCCESS' if res.success else 'FAILED'}")
        print(res.message)
        press_enter()

    def action_show_status(self):
        """Show which arm groups are enabled/disabled."""
        _, joints = self.get_joints_raw()

        # Determine group status: a group is "disabled" if ALL its joints are locked
        arm_l_joints = [n for n, _ in joints if n.startswith('arm_l')]
        arm_r_joints = [n for n, _ in joints if n.startswith('arm_r')]
        lift_joints  = [n for n, _ in joints if n.startswith('lift')]

        def group_status(group_joints, label):
            locked = sum(1 for n, l in joints if n in group_joints and l)
            total = len(group_joints)
            if total == 0:
                return f"{label}: \033[90munknown\033[0m"
            if locked == total:
                return f"{label}: \033[31mDISABLED\033[0m (all {total} joints locked)"
            elif locked == 0:
                return f"{label}: \033[32mENABLED\033[0m (all {total} joints free)"
            else:
                return f"{label}: \033[33mPARTIAL\033[0m ({locked}/{total} joints locked)"

        clear_screen()
        print("=== Arm Group Status ===\n")
        print(group_status(lift_joints,  "Lift"))
        print(group_status(arm_l_joints, "Left arm"))
        print(group_status(arm_r_joints, "Right arm"))
        print()
        print("(Group status is derived from individual joint lock state.)")
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

    def action_list_poses(self):
        """List all saved poses."""
        clear_screen()
        print("=== Poses in Memory ===\n")
        poses = self.call_list_memory()
        if poses:
            for p in poses:
                print(f"  - {p}")
        else:
            print("  (none)")

        print("\n=== Poses in File ===\n")
        filepath = POSES_FILE
        if os.path.exists(filepath):
            file_poses = []
            with open(filepath, 'r') as f:
                for line in f:
                    if line.startswith('[') and line.endswith(']\n'):
                        file_poses.append(line[1:-2])
            if file_poses:
                for p in file_poses:
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
        print(f"\nResult: {'SUCCESS' if res.success else 'FAILED'}")
        print(res.message)
        press_enter()

    # ── Main loop ──────────────────────────────────────────────────

    def run(self):
        self.wait_for_services()

        main_options = [
            "List joints                         (show LOCKED/FREE status)",
            "Show kinematic tree",
            "Toggle joint lock                   (individual joints)",
            "Toggle arm group                    (left_arm / right_arm / lift)",
            "Show arm group status               (which groups enabled)",
            "Save pose to memory                 (temporary)",
            "Save pose to file                   (persistent)",
            "Load pose from memory               (temporary)",
            "Load pose from file                 (from poses.txt)",
            "List saved poses                    (memory + file)",
            "Reset to home                       (re-enable all, go home)",
            "Exit",
        ]

        while True:
            idx = select_menu(main_options, "IK Solver CLI")
            if idx is None or idx == len(main_options) - 1:
                clear_screen()
                print("Exiting. Goodbye!")
                break

            action_map = {
                0:  self.action_list_joints,
                1:  self.action_show_tree,
                2:  self.action_toggle_joint,
                3:  self.action_toggle_group,
                4:  self.action_show_status,
                5:  lambda: self.action_save_pose(to_file=False),
                6:  lambda: self.action_save_pose(to_file=True),
                7:  lambda: self.action_load_pose(from_file=False),
                8:  lambda: self.action_load_pose(from_file=True),
                9:  self.action_list_poses,
                10: self.action_reset_home,
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
    finally:
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
