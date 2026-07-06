#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import re

class IKInteractiveCLI(Node):
    def __init__(self):
        super().__init__('ik_interactive_cli')
        self.tree_client = self.create_client(Trigger, '/ik_solver/get_tree')
        self.list_client = self.create_client(Trigger, '/ik_solver/list_joints')
        self.lock_pub = self.create_publisher(String, '/teleop_locks', 10)
        
        while not self.tree_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK Solver services...')

    def get_joints_info(self):
        req = Trigger.Request()
        future = self.list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result().message
        
        joints = []
        for line in msg.split('\n'):
            match = re.search(r'\[\d+\] \[(.*?)\] (.*)', line)
            if match:
                is_locked = 'LOCKED' in match.group(1)
                jname = match.group(2).strip()
                joints.append((jname, is_locked))
        return msg, joints

    def run(self):
        print("\n=== Connected to Interactive IK CLI ===")
        
        while rclpy.ok():
            print("\nOptions:")
            print("1) list joints")
            print("2) show kinematic tree")
            print("3) toggle/lock/unlock joints")
            print("4) exit")
            
            try:
                choice = input("\nSelect option (1-4)> ").strip()
                if not choice:
                    continue
                
                if choice == '1':
                    msg, _ = self.get_joints_info()
                    print("\n" + msg)
                
                elif choice == '2':
                    req = Trigger.Request()
                    future = self.tree_client.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    print("\n" + future.result().message)
                
                elif choice == '3':
                    raw_input = input("\nEnter joint names or prefixes (separated by space or comma)> ").strip()
                    if not raw_input:
                        continue
                        
                    # Parse input
                    tokens = [t.strip() for t in re.split(r'[,\s]+', raw_input) if t.strip()]
                    
                    _, joints = self.get_joints_info()
                    
                    matched_joints = []
                    for t in tokens:
                        for jname, is_locked in joints:
                            if jname.startswith(t) or t == jname:
                                if jname not in matched_joints:
                                    matched_joints.append(jname)
                    
                    if not matched_joints:
                        print("No joints matched your input.")
                        continue
                        
                    print("\nThe following exact joints will be toggled:")
                    for j in matched_joints:
                        print(f"  - {j}")
                        
                    confirm = input("\nConfirm? (y/n)> ").strip().lower()
                    if confirm == 'y':
                        for jname in matched_joints:
                            msg = String()
                            msg.data = jname
                            self.lock_pub.publish(msg)
                        print("Sent toggle requests!")
                    else:
                        print("Cancelled.")
                
                elif choice == '4' or choice == 'exit':
                    break
                else:
                    print("Invalid choice. Please select 1, 2, 3, or 4.")
                    
            except (EOFError, KeyboardInterrupt):
                break

def main(args=None):
    rclpy.init(args=args)
    cli = IKInteractiveCLI()
    cli.run()
    cli.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
