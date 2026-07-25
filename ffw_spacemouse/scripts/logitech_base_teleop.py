#!/usr/bin/env python3
"""
logitech_base_teleop.py — Logitech Extreme 3D Pro base control.

Hold Trigger (btn 0) + Thumb (btn 1) on the Logitech for > 2 seconds to toggle
base control from the SpaceMouse to the Logitech.  When active, stick X/Y and
twist drive /cmd_vel with x³ scaling, and the throttle slider sets a global
speed multiplier.  The SpaceMouse base_joy_callback is suppressed via a shared
/logitech/base_active flag.

Only affects BASE mode — ARM mode is unchanged.

Usage (standalone):
    ros2 launch ffw_spacemouse logitech_teleop.launch.py

In unified launch:
    ros2 launch ffw_spacemouse spacemouse_unified_teleop.launch.py use_logitech:=true
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class LogitechBaseTeleop(Node):
    def __init__(self):
        super().__init__('logitech_base_teleop')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('joy_topic', '/joy_logitech')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mode_topic', '/teleop_mode')
        self.declare_parameter('active_topic', '/logitech/base_active')
        self.declare_parameter('max_linear_vel', 0.6)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('axis_x', 1)
        self.declare_parameter('axis_y', 0)
        self.declare_parameter('axis_yaw', 2)
        self.declare_parameter('axis_speed', 3)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('hold_duration', 2.0)

        # ── State ───────────────────────────────────────────────────────
        self.active = False
        self.hold_start = None  # timestamp when both buttons first pressed
        self.mode = 'BASE'      # tracked from /teleop_mode
        self._was_active = False

        # ── Subscribers ─────────────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy,
            self.get_parameter('joy_topic').value,
            self.joy_cb,
            10)

        self.mode_sub = self.create_subscription(
            String,
            self.get_parameter('mode_topic').value,
            self.mode_cb,
            10)

        # ── Publishers ──────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10)

        self.active_pub = self.create_publisher(
            Bool,
            self.get_parameter('active_topic').value,
            10)

        self.get_logger().info(
            'Logitech Base Teleop started — hold Trigger+Thumb 2s to toggle')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def mode_cb(self, msg: String):
        self.mode = msg.data

    def joy_cb(self, msg: Joy):
        buttons = msg.buttons
        now = self.get_clock().now().nanoseconds / 1e9

        # ── Hold-to-toggle: Trigger (0) + Thumb (1) > 2 seconds ──────
        both_pressed = len(buttons) > 1 and bool(buttons[0]) and bool(buttons[1])

        if both_pressed:
            if self.hold_start is None:
                self.hold_start = now
            elif now - self.hold_start >= self.get_parameter('hold_duration').value:
                self.active = not self.active
                self.hold_start = None
                self.get_logger().info(
                    f'Logitech base control {"ON" if self.active else "OFF"}')
                self._publish_active()
        else:
            self.hold_start = None

        # ── Publish cmd_vel when active AND in BASE mode ─────────────
        if self.active and self.mode == 'BASE':
            self._publish_cmd_vel(msg)
        elif not self.active:
            # Publish a single zero cmd_vel on deactivation edge
            if self._was_active:
                self._publish_zero()
            self._was_active = False
            return

        self._was_active = self.active

    # ── Internal helpers ───────────────────────────────────────────────────

    def _get_axis(self, msg: Joy, index: int, invert: bool = False) -> float:
        if 0 <= index < len(msg.axes):
            val = msg.axes[index]
            if abs(val) < 0.05:
                return 0.0
            return -val if invert else val
        return 0.0

    def _publish_cmd_vel(self, msg: Joy):
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value

        x = self._get_axis(msg,
                           self.get_parameter('axis_x').value,
                           self.get_parameter('invert_x').value)
        y = self._get_axis(msg,
                           self.get_parameter('axis_y').value,
                           self.get_parameter('invert_y').value)
        yaw = self._get_axis(msg,
                             self.get_parameter('axis_yaw').value,
                             self.get_parameter('invert_yaw').value)

        def apply(val, vmax):
            return (val * val * val * val * val) * vmax

        twist = Twist()
        twist.linear.x = apply(x, max_lin)
        twist.linear.y = apply(y, max_lin)
        twist.angular.z = apply(yaw, max_ang)

        self.cmd_vel_pub.publish(twist)

    def _publish_zero(self):
        zero = Twist()
        self.cmd_vel_pub.publish(zero)

    def _publish_active(self):
        msg = Bool()
        msg.data = self.active
        self.active_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LogitechBaseTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
