import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import math
from tf_transformations import quaternion_from_euler


class TestPosePublisher(Node):
    def __init__(self):
        super().__init__('test_pose_publisher')

        self.pose_pub_ = self.create_publisher(Pose, '/vr_hand_pose_l', 10)
        self.marker_pub_ = self.create_publisher(Marker, '/vr_hand_pose_marker_l', 10)

        self.pose_pub_r_ = self.create_publisher(Pose, '/vr_hand_pose_r', 10)
        self.marker_pub_r_ = self.create_publisher(Marker, '/vr_hand_pose_marker_r', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.time_elapsed = 0.0

        self.center_x = 0.6
        self.center_y = 0.3
        self.center_z = 1.0
        self.radius = 0.2
        self.period = 10.0

    def timer_callback(self):
        self.time_elapsed += 0.05
        theta = 2 * math.pi * (self.time_elapsed % self.period) / self.period

        roll_deg = 0.0
        pitch_rad =  0.0 #math.sin(self.time_elapsed) * 0.5
        yaw_rad = 0.0#math.sin(self.time_elapsed) * 0.5

        q_l = quaternion_from_euler(math.radians(-roll_deg), pitch_rad, yaw_rad)
        q_r = quaternion_from_euler(math.radians(roll_deg), pitch_rad, yaw_rad)

        pose_left = Pose()
        pose_left.position.x = self.center_x + self.radius * math.sin(theta)
        pose_left.position.y = self.center_y # self.radius * math.sin(theta)
        pose_left.position.z = self.center_z #+ self.radius * math.cos(theta)
        pose_left.orientation.x = q_l[0]
        pose_left.orientation.y = q_l[1]
        pose_left.orientation.z = q_l[2]
        pose_left.orientation.w = q_l[3]
        self.pose_pub_.publish(pose_left)

        pose_right = Pose()
        pose_right.position.x = self.center_x #+ self.radius * math.sin(theta)
        pose_right.position.y = self.center_y - 0.6 - self.radius * math.sin(theta)
        pose_right.position.z = self.center_z + self.radius * math.cos(theta)
        pose_right.orientation.x = q_r[0]
        pose_right.orientation.y = q_r[1]
        pose_right.orientation.z = q_r[2]
        pose_right.orientation.w = q_r[3]
        self.pose_pub_r_.publish(pose_right)

        marker_l = Marker()
        marker_l.header.frame_id = "base_link"
        marker_l.header.stamp = self.get_clock().now().to_msg()
        marker_l.ns = "vr_hand_l"
        marker_l.id = 0
        marker_l.type = Marker.ARROW
        marker_l.action = Marker.ADD
        marker_l.pose = pose_left
        marker_l.scale.x = 0.3
        marker_l.scale.y = 0.05
        marker_l.scale.z = 0.05
        marker_l.color.a = 1.0
        marker_l.color.r = 0.2
        marker_l.color.g = 0.8
        marker_l.color.b = 1.0
        self.marker_pub_.publish(marker_l)

        marker_r = Marker()
        marker_r.header.frame_id = "base_link"
        marker_r.header.stamp = self.get_clock().now().to_msg()
        marker_r.ns = "vr_hand_r"
        marker_r.id = 1
        marker_r.type = Marker.ARROW
        marker_r.action = Marker.ADD
        marker_r.pose = pose_right
        marker_r.scale.x = 0.3
        marker_r.scale.y = 0.05
        marker_r.scale.z = 0.05
        marker_r.color.a = 1.0
        marker_r.color.r = 1.0
        marker_r.color.g = 0.5
        marker_r.color.b = 0.2
        self.marker_pub_r_.publish(marker_r)

        self.get_logger().info(
            f"[ROLL] {roll_deg:.1f}° | [L] y={pose_left.position.y:.2f}, z={pose_left.position.z:.2f} | "
            f"[R] y={pose_right.position.y:.2f}, z={pose_right.position.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TestPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
