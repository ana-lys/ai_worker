#!/usr/bin/env python3
"""Monitor XM430-W350 gripper (ID 130) current from /dynamic_joint_states."""

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState


class GripperCurrentMonitor(Node):
    def __init__(self):
        super().__init__('gripper_current_monitor')
        self.sub = self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self.cb, 10)

    def cb(self, msg):
        for name, iv in zip(msg.joint_names, msg.interface_values):
            if name != 'dxl130':
                continue
            values = dict(zip(iv.interface_names, iv.values))
            current_ma = values.get('Present Current', None)
            pos = values.get('Present Position', None)
            vel = values.get('Present Velocity', None)
            voltage = values.get('Present Input Voltage', None)
            hw_err = values.get('Hardware Error Status', None)

            output = f"[XM430 ID 130] Current: {current_ma:>10.6f} mA"
            if pos is not None:
                output += f" | Pos: {pos:>10.6f}"
            if vel is not None:
                output += f" | Vel: {vel:>8.2f}"
            if voltage is not None:
                output += f" | Vin: {voltage:>5.2f} V"
            if hw_err is not None and hw_err != 0:
                output += f" | ⚠ HW_ERROR={int(hw_err)}"
            print(output)
            break


def main():
    rclpy.init()
    node = GripperCurrentMonitor()
    print("Monitoring XM430-W350 gripper current (ID 130)...")
    print("Open/close the gripper and watch the values.")
    print()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
