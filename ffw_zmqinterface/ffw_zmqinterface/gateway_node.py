"""Gateway between ZMQ (RL/controller side) and the local spacemouse teleop stack.

Runs on the machine attached to the robot. Bridges two worlds:

  ZMQ side (same HIL link as robot_node, so they are mutually exclusive):
    PUB tcp://*:STATE_PORT     -- EEState   (current 2 end-effector poses)
    SUB tcp://*:CONTROL_PORT   -- ControlCmd (command poses from the controller)

  ROS side (spacemouse teleop stack):
    sub /ik_solver/achieved_ee_pose_{r,l}  -- current achieved EE poses (map frame)
    pub /quest/{right,left}/ee_target_pose -- absolute goal poses for the solver

ee0 <-> right arm, ee1 <-> left arm (see protocol.py).

The command path feeds /quest/<arm>/ee_target_pose, the absolute-gesture path in
ffw_ik_solver_teleop. That path consumes a map-frame goal directly (leashed
~6 cm/tick), is NOT gated on the quest being active, and is the correct injection
point for remote commands -- the delta path /spacemouse/<arm>/ee_target_pose
treats a large jump as a mapper re-base and would swallow a distant command.

Every ControlCmd frame carries a sender timestamp in its header (protocol.py),
so re-asserting the same goal is byte-unique and passes the _last_pub_data
dedup -- the dedup only suppresses exact retransmissions of one frame. A
controller walking the solver's per-message leash (e.g. a 25 Hz re-assert loop)
gets every re-assertion forwarded, no payload jitter required.

When the controller stops sending, the gateway stops publishing after
--cmd-timeout and the solver holds its last quest goal. Moving the spacemouse
re-engages the delta path (joy_hand publishes /spacemouse/<arm>/ee_target_pose).

Usage:
    ros2 run ffw_zmqinterface gateway_node --ros-args \
        -p state_port:=6001 -p control_port:=6002 -p hz:=100.0
"""

import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as proto


def _declare_float(node, name, default):
    """Declare a float param that also accepts integer overrides (e.g. -p hz:=100)."""
    try:
        node.declare_parameter(name, default)
    except rclpy.exceptions.InvalidParameterTypeException:
        node.declare_parameter(name, int(default))
    return float(node.get_parameter(name).value)


# protocol ee index -> arm/topic. ee0 = right, ee1 = left; flip the tuple to swap.
_CMD_TOPIC = {0: "/quest/right/ee_target_pose", 1: "/quest/left/ee_target_pose"}
_ACHIEVED_TOPIC = {0: "/ik_solver/achieved_ee_pose_r", 1: "/ik_solver/achieved_ee_pose_l"}


class GatewayNode(Node):
    def __init__(self):
        super().__init__("zmq_spacemouse_gateway")
        self.declare_parameter("state_port", 6001)
        self.declare_parameter("control_port", 6002)
        hz = _declare_float(self, "hz", 100.0)
        self._cmd_timeout = _declare_float(self, "cmd_timeout", 0.5)  # s; stop forwarding after this gap
        state_port = self.get_parameter("state_port").value
        control_port = self.get_parameter("control_port").value

        # Latest data, shared with the ZMQ receive thread. Guarded by _lock.
        self._lock = threading.Lock()
        self._achieved = [None, None]  # index 0=right(ee0), 1=left(ee1)
        self._cmd = None               # newest ControlCmd
        self._cmd_ts = 0.0             # sender timestamp from the frame header
        self._cmd_recv_ts = 0.0        # monotonic time it arrived (timeout basis)
        self._last_pub_data = None     # dedup: last full frame bytes we forwarded

        # ZMQ: state out (PUB latest-wins), control in (SUB keep-newest).
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.setsockopt(zmq.CONFLATE, 1)
        self._pub.bind(f"tcp://*:{state_port}")
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.CONFLATE, 1)
        self._sub.bind(f"tcp://*:{control_port}")

        # ROS: current poses in, goal poses out.
        for i, topic in _ACHIEVED_TOPIC.items():
            self.create_subscription(
                PoseStamped, topic, lambda msg, idx=i: self._on_achieved(idx, msg), 10
            )
        self._quest_pubs = {
            i: self.create_publisher(PoseStamped, topic, 10)
            for i, topic in _CMD_TOPIC.items()
        }

        self._recv_thread = threading.Thread(
            target=self._recv_loop, name="zmq-control-recv", daemon=True
        )
        self._recv_thread.start()

        period = 1.0 / hz
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"gateway: EEState-> {state_port} (PUB), ControlCmd<- {control_port} (SUB), "
            f"{hz:.0f} Hz, cmd timeout {self._cmd_timeout:.1f} s"
        )

    # -- receive thread: ZMQ SUB only --------------------------------

    def _recv_loop(self):
        while True:
            try:
                data = self._sub.recv()
            except zmq.ZMQError:
                break
            try:
                cmd, ts = proto.decode_control(data)
            except ValueError:
                self.get_logger().warn("dropped malformed ControlCmd")
                continue
            with self._lock:
                self._cmd = cmd
                self._cmd_ts = ts          # sender clock, kept for re-encode
                self._cmd_recv_ts = time.monotonic()

    # -- executor thread: ROS subs + timer only ----------------------

    def _on_achieved(self, idx, msg):
        with self._lock:
            self._achieved[idx] = msg

    def _tick(self):
        with self._lock:
            achieved = list(self._achieved)
            cmd = self._cmd
            cmd_ts = self._cmd_ts
            cmd_recv_ts = self._cmd_recv_ts
        now = time.monotonic()

        # State out: both arms as one EEState (quat -> rpy on the wire).
        if achieved[0] is not None and achieved[1] is not None:
            ee = []
            for msg in achieved:
                p = msg.pose
                rx, ry, rz = proto.quat_to_rpy(
                    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
                )
                ee.append((p.position.x, p.position.y, p.position.z, rx, ry, rz))
            if all(math.isfinite(v) for arm in ee for v in arm):
                self._pub.send(proto.encode_ee_state(proto.EEState(tuple(ee))))

        # Command in: newest ControlCmd -> /quest/<arm>/ee_target_pose.
        if cmd is not None and (now - cmd_recv_ts) < self._cmd_timeout:
            self._forward_cmd(cmd, cmd_ts)

    def _forward_cmd(self, cmd, ts):
        vals = cmd.ee[0] + cmd.ee[1]
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().warn("dropped non-finite ControlCmd")
            return
        data = proto.encode_control(cmd, ts=ts)
        if data == self._last_pub_data:
            return  # identical frame (type+ts+payload) already forwarded
        for i in (0, 1):
            x, y, z, rx, ry, rz = cmd.ee[i]
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            w, qx, qy, qz = proto.rpy_to_quat(rx, ry, rz)
            pose.pose.orientation.w = w
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            self._quest_pubs[i].publish(pose)
        self._last_pub_data = data


def main():
    rclpy.init()
    node = GatewayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
