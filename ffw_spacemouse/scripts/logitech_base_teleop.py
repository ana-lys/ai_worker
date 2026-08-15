#!/usr/bin/env python3
"""
logitech_base_teleop.py — Logitech Extreme 3D Pro full control.

Base control via stick + twist, head control via hat (edge-triggered),
elevation via buttons (2-goal — each button sends a fixed joint position).

Hold Trigger (btn 0) + Thumb (btn 1) on the Logitech for > 2 seconds to toggle
base control from the SpaceMouse to the Logitech.  When active, stick X/Y and
twist drive /cmd_vel with quadratic deadzone scaling.

Head control (hat) and elevation (buttons) work whenever the node is in BASE
mode, independent of the base-active toggle.

Head uses edge-triggered hat detection — one head_step per physical press
(0→±1 transition), NOT continuous integration.  This is necessary because the
hat is digital (±1 or 0) and joy_node autorepeat sends the held value at
100 Hz; continuous integration would race to joint limits in ~0.1 seconds.

Elevation uses 2 fixed goal offsets — each button press publishes on /left/joy
with axes[2] set so the IK solver's existing joy_callback_l applies the offset
via the same z^3*0.001 formula the SpaceMouse uses.

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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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

        # Head parameters (hat-goal analog to joy_base)
        self.declare_parameter('head_topic',
                               '/leader/joystick_controller_left/joint_trajectory')
        self.declare_parameter('head_step', 0.05)
        self.declare_parameter('head_joint1_lower', -0.2317)
        self.declare_parameter('head_joint1_upper', 0.6951)
        self.declare_parameter('head_joint2_lower', -0.35)
        self.declare_parameter('head_joint2_upper', 0.35)
        self.declare_parameter('axis_hat_x', 4)
        self.declare_parameter('axis_hat_y', 5)

        # Elevation parameters (2-goal — Z offsets via /left/joy, same topic + same
        # joy_callback_l the SpaceMouse uses, same z^3*0.001 formula)
        self.declare_parameter('elevation_joy_topic', '/left/joy')
        self.declare_parameter('elevation_goal_a', -0.1)   # lower both EEs by 10 cm
        self.declare_parameter('elevation_goal_b', 0.15)   # raise both EEs by 15 cm
        self.declare_parameter('btn_goal_a', 5)
        self.declare_parameter('btn_goal_b', 6)

        # ── State ───────────────────────────────────────────────────────
        self.active = False
        self.hold_start = None
        self.mode = 'BASE'
        self._was_active = False

        # Head integrated position (tilt, pan)
        self.current_head_pos = [0.0, 0.0]

        # Edge detection for hat (prevents continuous integration at 100 Hz)
        self._prev_hat_x = 0.0
        self._prev_hat_y = 0.0

        # ── Subscribers ─────────────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy, self.get_parameter('joy_topic').value, self.joy_cb, 10)
        self.mode_sub = self.create_subscription(
            String, self.get_parameter('mode_topic').value, self.mode_cb, 10)

        # ── Publishers ──────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.active_pub = self.create_publisher(
            Bool, self.get_parameter('active_topic').value, 10)
        self.head_pub = self.create_publisher(
            JointTrajectory, self.get_parameter('head_topic').value, 10)
        self.elevation_pub = self.create_publisher(
            Joy, self.get_parameter('elevation_joy_topic').value, 10)

        self.get_logger().info(
            'Logitech Base Teleop started — hold Trigger+Thumb 2s to toggle, '
            'hat→head (edge-triggered), btns 5/6→2-goal elevation')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def mode_cb(self, msg: String):
        self.mode = msg.data

    def joy_cb(self, msg: Joy):
        buttons = msg.buttons
        now = self.get_clock().now().nanoseconds / 1e9

        # ── Hold-to-toggle: Trigger (0) + Thumb (1) > 2 seconds ────────
        both_pressed = (len(buttons) > 1 and
                        bool(buttons[0]) and bool(buttons[1]))

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

        # ── Publish cmd_vel when active AND in BASE mode ───────────────
        if self.active and self.mode == 'BASE':
            self._publish_cmd_vel(msg)
        elif not self.active and self._was_active:
            self._publish_zero()    # deactivation edge → one zero cmd_vel

        self._was_active = self.active

        # ── Head and elevation work in BASE mode, independent of active ─
        if self.mode == 'BASE':
            self._process_head(msg)
            self._process_elevation(msg)

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

        x = self._get_axis(msg, self.get_parameter('axis_x').value,
                           self.get_parameter('invert_x').value)
        y = self._get_axis(msg, self.get_parameter('axis_y').value,
                           self.get_parameter('invert_y').value)
        yaw = self._get_axis(msg, self.get_parameter('axis_yaw').value,
                             self.get_parameter('invert_yaw').value)

        def apply(val, vmax):
            s = max(val * val - 0.04, 0.0) * 1.04
            return (-s if val < 0 else s) * vmax

        twist = Twist()
        twist.linear.x = apply(x, max_lin)
        twist.linear.y = apply(y, max_lin)
        twist.angular.z = apply(yaw, max_ang)
        self.cmd_vel_pub.publish(twist)

    def _publish_zero(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_active(self):
        msg = Bool()
        msg.data = self.active
        self.active_pub.publish(msg)

    # ── Head control (hat → head_joint1 tilt, head_joint2 pan) ────────────
    #
    # Hat is DIGITAL (±1 or 0) — every held frame at 100 Hz autorepeat
    # sends the same ±1 value.  Continuous integration would race the head
    # to joint limits in ~0.1 s (0.05 rad/tick × 100 ticks/s = 5 rad/s for
    # a ±0.93 rad range).
    #
    # Instead we use EDGE TRIGGERING: detect the 0→±1 transition and apply
    # exactly one head_step per physical press.  No movement while held,
    # no dead-stop publish on release.
    #
    # Currently no cubing (unlike the SpaceMouse analog axes) because the
    # hat value is always ±1.  A future "hold for repeat" mode could use
    # the 100 Hz autorepeat with a smaller effective step.
    # ───────────────────────────────────────────────────────────────────────

    def _process_head(self, msg: Joy):
        hat_x = self._get_axis(msg, self.get_parameter('axis_hat_x').value)
        hat_y = self._get_axis(msg, self.get_parameter('axis_hat_y').value)
        head_step = self.get_parameter('head_step').value

        publish_needed = False

        # ── Hat Y edge: head_joint1 (tilt) — NEGATE for correct direction ─
        if self._is_hat_pressed(self._prev_hat_y, hat_y):
            # Negate hat_y: pushing hat UP (>0) → head_joint1 INCREASES (tilt up)
            self.current_head_pos[0] = self._clamped_pos(
                0, self.current_head_pos[0] + (-hat_y) * head_step)
            publish_needed = True

        # ── Hat X edge: head_joint2 (pan) ─────────────────────────────
        if self._is_hat_pressed(self._prev_hat_x, hat_x):
            self.current_head_pos[1] = self._clamped_pos(
                1, self.current_head_pos[1] + hat_x * head_step)
            publish_needed = True

        self._prev_hat_x = hat_x
        self._prev_hat_y = hat_y

        if publish_needed:
            traj = JointTrajectory()
            traj.joint_names = ['head_joint1', 'head_joint2']
            pt = JointTrajectoryPoint()
            pt.time_from_start.nanosec = 50000000  # 50 ms
            pt.positions = [self.current_head_pos[0], self.current_head_pos[1]]
            pt.velocities = [0.0, 0.0]
            traj.points.append(pt)
            self.head_pub.publish(traj)

    def _is_hat_pressed(self, prev: float, curr: float) -> bool:
        """True on 0→±1 transition.  (±1→±1 stays False = no repeat.)"""
        return abs(prev) < 0.5 and abs(curr) >= 0.5

    def _clamped_pos(self, joint_idx: int, val: float) -> float:
        """Clamp val to the joint limits for head_joint1 (0) or head_joint2 (1)."""
        keys = ['head_joint1_lower', 'head_joint2_lower',
                'head_joint1_upper', 'head_joint2_upper']
        lo = self.get_parameter(keys[joint_idx]).value
        hi = self.get_parameter(keys[joint_idx + 2]).value
        return max(lo, min(hi, val))

    # ── Elevation control (2-goal — Z offsets for both end effectors) ────────
    #
    # Publishes on /left/joy with axes[2] set to a Z value that, when processed
    # by the IK solver's EXISTING joy_callback_l, produces the desired offset:
    #
    #   z_delta = z^3 * 0.001  →  z = (goal_offset * 1000)^(1/3)
    #
    # Example: goal_b = +0.15 m → z = (150)^(1/3) ≈ 5.31
    #          goal_a = -0.10 m → z = (-100)^(1/3) ≈ -4.64
    #
    # Each press adds the offset via the same `+=` accumulation that the
    # SpaceMouse Z axis uses — no new C++ code, no new prints.
    #
    # Button 5 (Top-4) → goal A, Button 6 (Top-5) → goal B.
    # Edge-triggered: one publish per physical press.
    # ──────────────────────────────────────────────────────────────────────────

    def _process_elevation(self, msg: Joy):
        buttons = msg.buttons
        btn_a = self.get_parameter('btn_goal_a').value
        btn_b = self.get_parameter('btn_goal_b').value
        goal_a = self.get_parameter('elevation_goal_a').value
        goal_b = self.get_parameter('elevation_goal_b').value

        # Edge detection state (init on first call)
        if not hasattr(self, '_prev_goal_a'):
            self._prev_goal_a = False
            self._prev_goal_b = False

        a_pressed = len(buttons) > btn_a and bool(buttons[btn_a])
        b_pressed = len(buttons) > btn_b and bool(buttons[btn_b])

        target_z = None
        if a_pressed and not self._prev_goal_a:
            target_z = goal_a
        elif b_pressed and not self._prev_goal_b:
            target_z = goal_b

        self._prev_goal_a = a_pressed
        self._prev_goal_b = b_pressed

        if target_z is not None:
            # Solve: z^3 * 0.001 = target_z  →  z = (target_z * 1000)^(1/3)
            # Handle negative base: Python float ** float returns complex for negatives
            z = abs(target_z * 1000.0) ** (1.0 / 3.0)
            if target_z < 0:
                z = -z
            joy_msg = Joy()
            joy_msg.axes = [0.0] * max(8, len(msg.axes))
            joy_msg.axes[2] = z
            joy_msg.buttons = []
            self.elevation_pub.publish(joy_msg)


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
