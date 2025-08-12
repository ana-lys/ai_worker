import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations

import socket
import threading
import json
import math
from rclpy.logging import LoggingSeverity, set_logger_level


class VRTCPBridge(Node):
    def __init__(self):
        super().__init__('vr_tcp_bridge')
        set_logger_level(self.get_logger().name, LoggingSeverity.DEBUG)

        self.left_pose_pub = self.create_publisher(Pose, '/vr_hand_pose', 10)
        self.right_pose_pub = self.create_publisher(Pose, '/vr_hand_pose_r', 10)
        self.left_grip_pub = self.create_publisher(Bool, '/vr/left_hand/grip', 10)
        self.right_grip_pub = self.create_publisher(Bool, '/vr/right_hand/grip', 10)
        self.left_joy_pub = self.create_publisher(Vector3, '/vr/left_hand/joystick', 10)
        self.right_joy_pub = self.create_publisher(Vector3, '/vr/right_hand/joystick', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/vr/hand_markers', 10)

        self.server_ip = '0.0.0.0'
        self.server_port = 5555
        threading.Thread(target=self.start_tcp_server, daemon=True).start()
        self.get_logger().info(f"🟢 TCP server listening on {self.server_ip}:{self.server_port}")
        self.center_x = 0.5
        self.center_y = 0.0
        self.center_z = 1.0
        self.initial_offsets = {
            'left': None,
            'right': None
        }

    def start_tcp_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((self.server_ip, self.server_port))
            server_socket.listen(1)
            while rclpy.ok():
                conn, addr = server_socket.accept()
                self.get_logger().info(f" Connected by {addr}")
                threading.Thread(target=self.handle_client, args=(conn,), daemon=True).start()

    def handle_client(self, conn):
        with conn:
            buffer = ''
            while True:
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    buffer += data.decode()
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self.handle_message(line.strip())
                except Exception as e:
                    self.get_logger().error(f' Error while receiving data: {e}')
                    break

    def handle_message(self, message):
        try:
            data = json.loads(message)
            # self.get_logger().debug(f" Parsed JSON: {data}")

            left_markers = self.process_hand('left', data['left'])
            right_markers = self.process_hand('right', data['right'])
            merged = MarkerArray()
            merged.markers.extend(left_markers.markers)
            merged.markers.extend(right_markers.markers)
            self.marker_pub.publish(merged)
        except Exception as e:
            self.get_logger().warn(f"⚠️ JSON parse error: {e}")
            self.get_logger().warn(f"⚠️ Failed message: {message}")

    def is_valid_quaternion(self, q):
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        return 0.9 < norm < 1.1

    def process_hand(self, side, hand):
        raw_x = hand['px'] * 1.1
        raw_y = hand['py'] * 1.0
        raw_z = hand['pz']

        if self.initial_offsets[side] is None:
            self.initial_offsets[side] = (raw_x, raw_y, raw_z)
            self.get_logger().info(f" Initial offset for {side} set to: {self.initial_offsets[side]}")

        init_x, init_y, init_z = self.initial_offsets[side]
        pose = Pose()
        pose.position.x = raw_x - init_x + self.center_x
        y_offset = 0.1 if side == 'left' else -0.1
        pose.position.y = raw_y - init_y + self.center_y + y_offset
        pose.position.z = raw_z - init_z + self.center_z

        pose.orientation.x = hand['ox']
        pose.orientation.y = hand['oy']
        pose.orientation.z = hand['oz']
        pose.orientation.w = hand['ow']

        grip_msg = Bool()
        grip_msg.data = hand['grip']

        joy_msg = Vector3()
        joy_msg.x = hand['joyx']
        joy_msg.y = hand['joyy']
        joy_msg.z = 0.0

        if side == 'left':
            self.left_pose_pub.publish(pose)
            self.left_grip_pub.publish(grip_msg)
            self.left_joy_pub.publish(joy_msg)
        else:
            self.right_pose_pub.publish(pose)
            self.right_grip_pub.publish(grip_msg)
            self.right_joy_pub.publish(joy_msg)

        return self.create_axes_arrows(pose, side)

    def create_axes_arrows(self, pose, side):
        base_id = 10 if side == 'left' else 20
        color_map = {
            'x': (1.0, 0.0, 0.0),
            'y': (0.0, 1.0, 0.0),
            'z': (0.0, 0.0, 1.0),
        }

        origin = pose.position
        quat = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        marker_array = MarkerArray()

        if not self.is_valid_quaternion(quat):
            self.get_logger().warn(f" Invalid quaternion: {quat}")
            return marker_array

        try:
            rot_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]
        except Exception as e:
            self.get_logger().warn(f" Quaternion to matrix error: {e}")
            return marker_array

        for i, axis in enumerate(['x', 'y', 'z']):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"{side}_axes"
            marker.id = base_id + i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            start = [origin.x, origin.y, origin.z]
            direction = rot_matrix[:, i]
            length = 0.15
            end = [start[j] + direction[j] * length for j in range(3)]
            marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
            marker.points.append(Point(x=end[0], y=end[1], z=end[2]))
            marker.scale.x = 0.01
            marker.scale.y = 0.02
            marker.scale.z = 0.0
            r, g, b = color_map[axis]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200_000_000
            marker_array.markers.append(marker)

        return marker_array


def main():
    rclpy.init()
    node = VRTCPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
