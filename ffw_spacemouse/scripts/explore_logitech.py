#!/usr/bin/env python3
"""
explore_logitech.py — Explore Logitech Extreme 3D Pro inputs.

Subscribes to /joy_logitech (isolated from SpaceMouse /joy) and prints
the full multi-line display.  Uses cursor-home + per-line erase (\x1b[K)
to overwrite in-place — no screen clear, no scrolling, no flash.

Rebuild after changes:

    cd ~/robotis_ws && colcon build --packages-select ffw_spacemouse

Usage:
    ros2 launch ffw_spacemouse explore_logitech.launch.py
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


# Axis mapping for Logitech Extreme 3D Pro on Linux:
#   0  X stick L/R
#   1  Y stick F/B
#   2  Z rotate (twist)
#   3  Throttle slider
#   4  Hat X
#   5  Hat Y
AXIS_LABELS = {
    0: 'X stick L/R',
    1: 'Y stick F/B',
    2: 'TW (twist)',
    3: 'THR (throttle)',
    4: 'HX (hat X)',
    5: 'HY (hat Y)',
}

BUTTON_LABELS = {
    0: 'Trigger',
    1: 'Thumb',
    2: 'Top-1',
    3: 'Top-2',
    4: 'Top-3',
    5: 'Top-4',
    6: 'Top-5',
    7: 'Top-6',
    8: 'Base-1',
    9: 'Base-2',
    10: 'Base-3',
    11: 'Base-4',
}


class LogitechExplorer(Node):
    def __init__(self):
        super().__init__('logitech_explorer')
        self.joy = Joy()
        self.received = False
        self._first = True

        self.sub = self.create_subscription(Joy, '/joy_logitech', self.cb, 10)
        self.timer = self.create_timer(0.01, self.tick)  # 100 Hz

        print('[joy_log] WAITING for /joy_logitech data ...', flush=True)

    # ── helpers ──────────────────────────────────────────────────────────

    def _bar(self, val, width=32):
        pos = int((val + 1.0) / 2.0 * width)
        pos = max(0, min(width, pos))
        return '█' * pos + '░' * (width - pos)

    def _sep(self, ch='─', n=72):
        return ch * n

    # ── callbacks ────────────────────────────────────────────────────────

    def cb(self, msg: Joy):
        self.joy = msg
        self.received = True

    def tick(self):
        if not self.received:
            return

        msg = self.joy
        t = time.strftime('%H:%M:%S.%f')[:-3]
        lines = []

        if self._first:
            lines.append('\x1b[2J\x1b[H')  # clear screen once at start
            self._first = False
        else:
            lines.append('\x1b[H')  # cursor home, overwrite in place

        lines.append(f'── Logitech Extreme 3D Pro  |  {t}  |  '
                     f'{len(msg.axes)} axes  {len(msg.buttons)} buttons  ──')

        # Axes — one per line with bar
        for i in range(min(len(msg.axes), 6)):
            label = AXIS_LABELS.get(i, f'Axis {i}')
            val = msg.axes[i]
            bar = self._bar(val)
            lines.append(f'  [{i}] {label:15s} {val:+7.4f}  {bar}')

        # Hat
        if len(msg.axes) >= 6:
            hx, hy = msg.axes[4], msg.axes[5]
            if abs(hx) > 0.5 or abs(hy) > 0.5:
                d = ''
                if hy > 0.5:
                    d += '↑'
                elif hy < -0.5:
                    d += '↓'
                if hx > 0.5:
                    d += '→'
                elif hx < -0.5:
                    d += '←'
                lines.append(f'  Hat: {d}  ({hx:+.2f}, {hy:+.2f})')
            else:
                lines.append(f'  Hat: · center  ({hx:+.2f}, {hy:+.2f})')

        # Buttons — grid of 4 per row
        lines.append('')
        lines.append(f'  Buttons ({len(msg.buttons)} total):')
        cols = 4
        for row in range(0, len(msg.buttons), cols):
            parts = []
            for j in range(row, min(row + cols, len(msg.buttons))):
                label = BUTTON_LABELS.get(j, f'B{j}')
                dot = '●' if msg.buttons[j] else '○'
                parts.append(f'[{j:2d}] {label:10s} {dot}')
            lines.append('    ' + '    '.join(parts))

        lines.append('')
        lines.append(self._sep('─'))

        # \x1b[K at the end of each line erases any leftover chars from prior frame
        for L in lines:
            print(f'[joy_log] {L}\x1b[K', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = LogitechExplorer()
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
