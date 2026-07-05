#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

class IKInteractiveCLI(Node):
    def __init__(self):
        super().__init__('ik_interactive_cli')
        self.tree_client = self.create_client(Trigger, '/ik_solver/get_tree')
        self.list_client = self.create_client(Trigger, '/ik_solver/list_joints')
        self.lock_pub = self.create_publisher(String, '/teleop_locks', 10)
        
        while not self.tree_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK Solver services...')

    def run(self):
        print("\n=== Connected to Interactive IK CLI ===")
        print("Type 'list' to see joints, 'tree' to see kinematic tree.")
        
        while rclpy.ok():
            try:
                cmd = input("\n> ").strip()
                if not cmd:
                    continue
                
                if cmd == 'tree':
                    req = Trigger.Request()
                    future = self.tree_client.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    print(future.result().message)
                
                elif cmd == 'list':
                    req = Trigger.Request()
                    future = self.list_client.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    print(future.result().message)
                
                else:
                    # Treat anything else as a toggle command
                    msg = String()
                    msg.data = cmd
                    self.lock_pub.publish(msg)
                    print(f"Sent toggle request for '{cmd}'")
                    
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
