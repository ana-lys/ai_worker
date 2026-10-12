#!/usr/bin/env python3
"""Watchdog for the published Quest controller poses.

Diagnoses "oscillating" EE motion at its source: subscribes to
/left_controller_pose and /right_controller_pose (quest_to_ros2.py's raw
output, before joy_hand's leash/interpolation) and reports, every
CHECK_PERIOD_S:

  1. Publisher count on each topic -- more than one publisher means two
     quest_to_ros2.py processes are racing on the same UDP port (each seeing
     only some packets, each with its own independent head-yaw reference),
     which produces exactly the kind of large, inconsistent jumps that look
     like oscillation. This is a process-hygiene bug, not sensor noise, and
     a low-pass filter would only mask it -- kill the duplicate instead:
         ps aux | grep quest_to_ros2.py
         kill <the older/orphaned PID>
  2. Statistical jitter on each position/rotation axis: zero-crossing rate
     of velocity (sign flips = back-and-forth motion, the signature of
     tracking noise) and the RMS of the high-frequency residual (raw minus a
     slow EMA). This is the check that actually justifies a low-pass filter.
  3. Discontinuity count: single-tick jumps exceeding a plausible max hand
     speed -- symptomatic of #1, or of a torso-relative re-anchor (head pose
     jump) in quest_to_ros2.py.

Usage:
    ros2 run ffw_spacemouse quest_pose_watchdog.py
    python3 quest_pose_watchdog.py --window 30 --period 2.0
"""

from __future__ import annotations

import argparse
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped

TOPICS = ("left_controller_pose", "right_controller_pose")

# Above this, a single-tick position jump is not plausible hand motion at
# 100 Hz (0.05 m / 10 ms = 5 m/s) -- flag as a discontinuity instead of
# folding it into the jitter RMS, which would hide it under legit-looking
# noise.
MAX_JUMP_M = 0.05

# Jitter verdict thresholds (tuned to be visibly worse than normal hand
# tremor -- Quest 3 Touch tracking noise is usually sub-millimeter at rest).
JITTER_POS_RMS_WARN_M = 0.003     # 3 mm
JITTER_ROT_RMS_WARN_RAD = 0.017   # ~1 deg
ZERO_CROSS_WARN_RATIO = 0.5       # >50% of consecutive velocity samples flip sign

EMA_ALPHA = 0.05  # slow reference for the high-frequency residual


def quat_angle_diff(q1, q2) -> float:
    """Angle (rad) of the relative rotation q1^-1 * q2, q as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    # q1_inv * q2 (unit quats: inverse == conjugate)
    w = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2
    return 2.0 * math.acos(min(1.0, abs(w)))


class AxisStats:
    """Rolling jitter stats for one scalar signal."""

    def __init__(self, window: int):
        self.window = window
        self.ema = None
        self.residuals = deque(maxlen=window)
        self.vel_signs = deque(maxlen=window)
        self.prev_val = None
        self.prev_vel = None
        self.max_jump = 0.0

    def update(self, val: float, dt: float):
        if self.prev_val is not None and dt > 0:
            delta = val - self.prev_val
            self.max_jump = max(self.max_jump, abs(delta))
            vel = delta / dt
            if self.prev_vel is not None:
                sign_flip = (vel * self.prev_vel) < 0.0
                self.vel_signs.append(1 if sign_flip else 0)
            self.prev_vel = vel
        self.prev_val = val

        self.ema = val if self.ema is None else (
            EMA_ALPHA * val + (1.0 - EMA_ALPHA) * self.ema)
        self.residuals.append(val - self.ema)

    def rms(self) -> float:
        if not self.residuals:
            return 0.0
        return math.sqrt(sum(r * r for r in self.residuals) / len(self.residuals))

    def zero_cross_ratio(self) -> float:
        if not self.vel_signs:
            return 0.0
        return sum(self.vel_signs) / len(self.vel_signs)


class SideMonitor:
    """Tracks one controller's pose stream: 3 position axes + 1 rotation axis."""

    def __init__(self, window: int):
        self.pos_axes = [AxisStats(window) for _ in range(3)]
        self.rot = AxisStats(window)
        self.last_stamp = None
        self.last_quat = None
        self.max_jump_this_period = 0.0
        self.msg_count = 0

    def on_msg(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_stamp is None else max(0.0, t - self.last_stamp)
        self.last_stamp = t
        self.msg_count += 1

        p = msg.pose.position
        for axis, val in zip(self.pos_axes, (p.x, p.y, p.z)):
            axis.update(val, dt)

        q = (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w)
        if self.last_quat is not None and dt > 0:
            ang = quat_angle_diff(self.last_quat, q)
            self.rot.update(ang, dt)  # treat accumulated angle as the scalar signal
        self.last_quat = q

        jump = math.sqrt(sum((a.max_jump) ** 2 for a in self.pos_axes))
        self.max_jump_this_period = max(self.max_jump_this_period, jump)
        for a in self.pos_axes:
            a.max_jump = 0.0  # consumed into jump above

    def report(self) -> str:
        pos_rms = max(a.rms() for a in self.pos_axes)
        pos_zc = max(a.zero_cross_ratio() for a in self.pos_axes)
        rot_rms = self.rot.rms()
        rot_zc = self.rot.zero_cross_ratio()
        discontinuity = self.max_jump_this_period > MAX_JUMP_M
        jittery = (
            (pos_rms > JITTER_POS_RMS_WARN_M and pos_zc > ZERO_CROSS_WARN_RATIO) or
            (rot_rms > JITTER_ROT_RMS_WARN_RAD and rot_zc > ZERO_CROSS_WARN_RATIO)
        )
        verdict = "OK"
        if discontinuity:
            verdict = "DISCONTINUITY"
        elif jittery:
            verdict = "OSCILLATING"
        line = (
            f"msgs={self.msg_count:4d}  "
            f"pos_rms={pos_rms * 1000:5.2f}mm  pos_zero_cross={pos_zc * 100:5.1f}%  "
            f"rot_rms={math.degrees(rot_rms):5.2f}deg  rot_zero_cross={rot_zc * 100:5.1f}%  "
            f"max_jump={self.max_jump_this_period * 1000:6.1f}mm  -> {verdict}"
        )
        self.max_jump_this_period = 0.0
        self.msg_count = 0
        return line, verdict


class QuestPoseWatchdog(Node):
    def __init__(self, window: int, period: float):
        super().__init__("quest_pose_watchdog")
        self.period = period
        self.monitors = {topic: SideMonitor(window) for topic in TOPICS}
        self.subs = []
        for topic in TOPICS:
            self.subs.append(self.create_subscription(
                PoseStamped, topic,
                (lambda msg, t=topic: self.monitors[t].on_msg(msg)), 50))
        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f"quest_pose_watchdog watching {TOPICS}, "
            f"window={window} samples, report every {period}s")

    def _tick(self):
        any_bad = False
        for topic in TOPICS:
            n_pub = self.count_publishers(topic)
            line, verdict = self.monitors[topic].report()
            if n_pub != 1:
                any_bad = True
                self.get_logger().warn(
                    f"[{topic}] {n_pub} PUBLISHERS DETECTED (expected 1) -- "
                    f"this alone explains oscillation/inconsistent poses. "
                    f"Find and kill the duplicate: "
                    f"`ps aux | grep quest_to_ros2.py`, keep only the one "
                    f"belonging to your current `ros2 launch` session.")
            if verdict != "OK":
                any_bad = True
            self.get_logger().info(f"[{topic}] pub={n_pub}  {line}")
        if not any_bad:
            self.get_logger().info(
                "-- clean: single publisher, no discontinuities, "
                "jitter within normal tracking noise --")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", type=int, default=30,
                         help="Rolling sample window for jitter stats (default: 30).")
    parser.add_argument("--period", type=float, default=2.0,
                         help="Report interval in seconds (default: 2.0).")
    args = parser.parse_args()

    rclpy.init()
    node = QuestPoseWatchdog(args.window, args.period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
