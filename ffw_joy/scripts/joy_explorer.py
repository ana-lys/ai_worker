#!/usr/bin/env python3
"""
joy_explorer.py — Explore all joystick inputs and use Logitech 3D Pro
as an alternative right-hand EE controller.

Mode cycle (press Logitech button 0 / trigger):
    0 → NAV mode        (SpaceMouse controls base)
    1 → HAND mode       (SpaceMouse controls end-effector)
    2 → LOGITECH mode   (Logitech controls end-effector)
    (loops back to 0)

Logitech EE mapping:
    Axis 0 (stick X)    → EE linear.x
    Axis 1 (stick Y)    → EE linear.y
    Axis 3 (twist)      → EE linear.z
    Axis 2 (throttle)   → EE angular.y (pitch rate)
    Buttons             → placeholder (printed but not mapped)

Subscribes: /joy, /right/joy, /left/joy
Publishes:  /joy/ee_cmd (geometry_msgs/Twist) — only when in LOGITECH mode
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import sys

MODE_NAMES = ['NAV (SpaceMouse)', 'HAND (SpaceMouse)', 'LOGITECH joystick']

# Logitech G Extreme 3D Pro axis map
AX = {
    'x': 0, 'y': 1, 'throttle': 2, 'twist': 3,
    'hat_x': 4, 'hat_y': 5,
}
AXIS_LABELS = {0: 'X', 1: 'Y', 2: 'THR', 3: 'TW', 4: 'HX', 5: 'HY'}


class JoyExplorer(Node):
    def __init__(self):
        super().__init__('joy_explorer')
        self.mode = 0
        self.btn_prev = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0}

        # Subscribe to all joystick topics
        self.create_subscription(Joy, '/joy',        self.cb_joy,  10)
        self.create_subscription(Joy, '/right/joy',  self.cb_right, 10)
        self.create_subscription(Joy, '/left/joy',   self.cb_left,  10)

        # Publish EE twist commands (only in LOGITECH mode)
        self.ee_pub = self.create_publisher(Twist, '/joy/ee_cmd', 10)

        # Latest messages from each device
        self.joy = Joy()
        self.right = Joy()
        self.left = Joy()

        # Display timer (5 Hz — avoids terminal spam)
        self.create_timer(0.2, self.display)
        # EE command timer (20 Hz when in Logitech mode)
        self.create_timer(0.05, self.publish_ee)

        self._header_printed = False

    # ── callbacks ────────────────────────────────────────────

    def cb_joy(self, msg):
        self.joy = msg
        # Cycle mode on button 0 rising edge
        if len(msg.buttons) > 0:
            cur = msg.buttons[0]
            if cur == 1 and self.btn_prev[0] == 0:
                self.mode = (self.mode + 1) % len(MODE_NAMES)
            self.btn_prev[0] = cur

    def cb_right(self, msg):
        self.right = msg

    def cb_left(self, msg):
        self.left = msg

    # ── publishers ───────────────────────────────────────────

    def publish_ee(self):
        if self.mode != 2:
            return
        a = self.joy.axes
        if len(a) < 6:
            return
        cmd = Twist()
        cmd.linear.x  = a[AX['x']]             * 0.1
        cmd.linear.y  = a[AX['y']]             * 0.1
        cmd.linear.z  = a[AX['twist']]         * 0.1    # twist → Z
        cmd.angular.y = a[AX['throttle']]      * 0.5    # throttle → pitch rate
        self.ee_pub.publish(cmd)

    # ── terminal display ─────────────────────────────────────

    def _fmt_axes(self, axes, limit=8):
        parts = []
        for i, v in enumerate(axes[:limit]):
            label = AXIS_LABELS.get(i, str(i))
            parts.append(f'{label}:{v:+5.3f}')
        return ' '.join(parts)

    def _fmt_btns(self, btns):
        return ''.join(str(b) for b in btns[:12]).ljust(12)

    def display(self):
        if not self._header_printed:
            self._print_header()
            self._header_printed = True

        n = 0  # lines we'll write

        # Line: mode
        mode_str = f'  MODE [{self.mode}]: {MODE_NAMES[self.mode]}'
        if self.mode == 2:
            a = self.joy.axes
            if len(a) >= 6:
                mode_str += (
                    f'  →  X:{a[0]:+.3f}  Y:{a[1]:+.3f}  '
                    f'Z:{a[3]:+.3f}  pitch:{a[2]:+.3f}'
                )
        print(mode_str)
        n += 1

        for label, joy in [('LOGITECH', self.joy),
                           ('RIGHT-SM', self.right),
                           ('LEFT-SM',  self.left)]:
            if joy.axes or joy.buttons:  # has data
                axes = self._fmt_axes(joy.axes)
                btns = self._fmt_btns(joy.buttons)
                print(f'  [{label:9s}]  {axes}  |  btns:{btns}')
                n += 1

        # Blank filler to cover stale lines from previous frame
        print('  ' + '─' * 70)
        n += 1

        # Move cursor back so next frame overwrites
        sys.stdout.write(f'\x1b[{n}A')
        sys.stdout.flush()

    def _print_header(self):
        print('\n' + '═' * 72)
        print('  JOY EXPLORER — press Logitech trigger (btn 0) to cycle mode')
        print('═' * 72)
        print(f'  Modes:  0=NAV  1=HAND  2=LOGITECH-EE')
        print(f'  Axes:   X/Y → lin.xy   twist→lin.z   throttle→pitch rate')
        print(f'  Publishes /joy/ee_cmd (Twist) when mode=2')
        print('─' * 72)


def main(args=None):
    rclpy.init(args=args)
    node = JoyExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('\nExited.')


if __name__ == '__main__':
    main()
