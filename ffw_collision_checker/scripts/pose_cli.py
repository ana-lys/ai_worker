#!/usr/bin/env python3
import os
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from ffw_collision_checker.srv import SaveLoadPose
from std_srvs.srv import Trigger

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # Handle escape sequence for arrow keys
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return '\x1b[' + ch3
            return '\x1b'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def select_menu(options, title="Select an option:"):
    if not options:
        return None
    selected = 0
    while True:
        # Clear terminal screen
        sys.stdout.write("\033[H\033[J")
        sys.stdout.write(f"=== {title} ===\n\n")
        for idx, opt in enumerate(options):
            if idx == selected:
                sys.stdout.write(f"\033[36m➔  {opt}\033[0m\n")
            else:
                sys.stdout.write(f"   {opt}\n")
        sys.stdout.write("\nUse [Up/Down] arrows to move, [Enter] to select, [q]/[Esc]/[Ctrl+C] to return/exit.\n")
        sys.stdout.flush()

        ch = getch()
        if ch == '\x1b[A':  # Up arrow
            selected = (selected - 1) % len(options)
        elif ch == '\x1b[B':  # Down arrow
            selected = (selected + 1) % len(options)
        elif ch in ('\r', '\n'):  # Enter
            return selected
        elif ch.lower() == 'q' or ch == '\x1b' or ch == '\x03':  # q, Esc, or Ctrl+C
            return None

class PoseCLI(Node):
    def __init__(self):
        super().__init__('pose_cli')
        self.save_client = self.create_client(SaveLoadPose, '/ik_solver/save_pose')
        self.load_client = self.create_client(SaveLoadPose, '/ik_solver/load_pose')
        self.list_client = self.create_client(Trigger, '/ik_solver/list_saved_poses')
        self.config_dir = '/home/lys/robotis_ws/src/ai_worker/ffw_collision_checker/config'
        
        # Create config directory if it doesn't exist
        os.makedirs(self.config_dir, exist_ok=True)

    def wait_for_services(self):
        self.get_logger().info("Connecting to IK Solver services...")
        while not self.save_client.wait_for_service(timeout_sec=2.0):
            print("Waiting for /ik_solver/save_pose service to start...")
        while not self.load_client.wait_for_service(timeout_sec=2.0):
            print("Waiting for /ik_solver/load_pose service to start...")
        while not self.list_client.wait_for_service(timeout_sec=2.0):
            print("Waiting for /ik_solver/list_saved_poses service to start...")
        print("Connected to all services!")

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
        future = self.list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res.success or not res.message.strip():
            return []
        return [line.strip() for line in res.message.split('\n') if line.strip()]

    def run(self):
        self.wait_for_services()
        
        main_options = [
            "Save current pose to memory (temporary)",
            "Load pose from memory (temporary)",
            "Save current pose to file",
            "Load pose from file",
            "Exit"
        ]

        while True:
            selected_idx = select_menu(main_options, "Robot Pose Manager")
            if selected_idx is None or selected_idx == 4:
                print("\nExiting. Goodbye!")
                break
            
            # Save to memory
            if selected_idx == 0:
                print("\n=== Save current pose to memory ===")
                try:
                    name = input("Enter temporary pose name: ").strip()
                    if name:
                        success, msg = self.call_save(name, False)
                        print(f"\nResult: {'SUCCESS' if success else 'FAILED'}\n{msg}")
                    else:
                        print("Cancelled: Name cannot be empty.")
                except (KeyboardInterrupt, EOFError):
                    print("\nCancelled.")
                input("\nPress Enter to continue...")

            # Load from memory
            elif selected_idx == 1:
                poses = self.call_list_memory()
                if not poses:
                    print("\nNo poses found in memory.")
                    input("\nPress Enter to continue...")
                    continue
                
                selected_pose_idx = select_menu(poses, "Select pose to load from memory")
                if selected_pose_idx is not None:
                    target_pose = poses[selected_pose_idx]
                    success, msg = self.call_load(target_pose, False)
                    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}\n{msg}")
                    input("\nPress Enter to continue...")

            # Save to file
            elif selected_idx == 2:
                print("\n=== Save current pose to file ===")
                try:
                    name = input("Enter file name (key for poses.txt): ").strip()
                    if name:
                        success, msg = self.call_save(name, True)
                        print(f"\nResult: {'SUCCESS' if success else 'FAILED'}\n{msg}")
                    else:
                        print("Cancelled: Name cannot be empty.")
                except (KeyboardInterrupt, EOFError):
                    print("\nCancelled.")
                input("\nPress Enter to continue...")

            # Load from file
            elif selected_idx == 3:
                filepath = os.path.join(self.config_dir, "poses.txt")
                if not os.path.exists(filepath):
                    print("\nNo saved poses found in file (poses.txt does not exist).")
                    input("\nPress Enter to continue...")
                    continue
                
                # Parse poses.txt to find [pose_name] tags
                poses = []
                with open(filepath, 'r') as f:
                    for line in f:
                        if line.startswith('[') and line.endswith(']\n'):
                            poses.append(line[1:-2])
                
                if not poses:
                    print("\nNo saved poses found in poses.txt.")
                    input("\nPress Enter to continue...")
                    continue
                
                selected_file_idx = select_menu(poses, "Select pose to load from file")
                if selected_file_idx is not None:
                    target_pose = poses[selected_file_idx]
                    success, msg = self.call_load(target_pose, True)
                    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}\n{msg}")
                    input("\nPress Enter to continue...")

def main(args=None):
    rclpy.init(args=args)
    cli = PoseCLI()
    try:
        cli.run()
    except KeyboardInterrupt:
        pass
    finally:
        cli.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
