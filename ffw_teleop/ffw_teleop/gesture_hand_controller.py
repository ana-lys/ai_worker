import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HandPublisher(Node):

    def __init__(self):
        super().__init__('hand_publisher')
        self.left_joint_names = [
            "finger_l_joint1", "finger_l_joint2", "finger_l_joint3", "finger_l_joint4",
            "finger_l_joint5", "finger_l_joint6", "finger_l_joint7", "finger_l_joint8",
            "finger_l_joint9", "finger_l_joint10", "finger_l_joint11", "finger_l_joint12",
            "finger_l_joint13", "finger_l_joint14", "finger_l_joint15", "finger_l_joint16",
            "finger_l_joint17", "finger_l_joint18", "finger_l_joint19", "finger_l_joint20"
        ]

        self.right_joint_names = [
            "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
            "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
            "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
            "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
            "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"
        ]

        self.present_left_thumb_joints = np.zeros(4)
        self.present_right_thumb_joints = np.zeros(4)

        self.present_left_finger_joints = np.zeros(20)
        self.present_right_finger_joints = np.zeros(20)

        self.left_thumb_subscriber_ = self.create_subscription(
            JointState,
            '/left_thumb/joint_states',
            self.left_thumb_callback,
            10
        )

        self.right_thumb_subscriber_ = self.create_subscription(
            JointState,
            '/right_thumb/joint_states',
            self.right_thumb_callback,
            10
        )

        self.left_finger_subscriber_ = self.create_subscription(
            JointTrajectory,
            '/left_hand/joint_trajectory',
            self.left_finger_callback,
            10
        )

        self.right_finger_subscriber_ = self.create_subscription(
            JointTrajectory,
            '/right_hand/joint_trajectory',
            self.right_finger_callback,
            10
        )

        self.left_hand_publisher_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory', 10)
        self.right_hand_publisher_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory', 10)
        self.toggle_publisher_ = self.create_publisher(Bool, '/teleop_control/toggle', 10)

        self.timer_period = 0.01

        self.timer = self.create_timer(self.timer_period, self.timer_callback) # JointTrajectory
        self.left_hand_start = False
        self.right_hand_start = False

        self.toggle_t0_right = time.perf_counter()
        self.toggle_t1_right = time.perf_counter()
        self.toggle_t0_left = time.perf_counter()
        self.toggle_t1_left = time.perf_counter()

        self.toggle_time = 2.0
        self.toggle_threshold = 0.7

        self.toggle_control_right = False # teleoperation OFF - False (default), ON - True
        self.toggle_control_left = False # teleoperation OFF - False (default), ON - True
        self.toggle_count_right = False # timer start
        self.toggle_count_left = False # timer start
        self.toggle_release_right = True # toggle status False after switching to teleoperation - needs to release first before re-switch
        self.toggle_release_left = True # toggle status False after switching to teleoperation - needs to release first before re-switch

        self.toggle_control_both = False

        self.toggle_pose_right = np.array([
            -0.4, -0.4, 0.0, 0.0,
            0.1, 0.1, 0.2, 0.0,
            0.1, 1.3, 1.8, 1.0,
            0.2, 1.1, 1.2, 0.7,
            0.5, 0.1, 0.3, 0.0
        ])
        self.toggle_pose_left = np.array([
            0.2, 0.6, 0.0, 0.0,
            0.0, 0.6, 0.2, 0.0,
            0.0, 1.7, 1.7, 0.8,
            0.0, 1.7, 1.7, 0.8,
            -0.3, 0.5, 0.1, 0.0
        ])

    def timer_callback(self):
        if self.left_hand_start:
            left_msg = JointTrajectory()
            left_msg.joint_names = self.left_joint_names
            left_traj_point = JointTrajectoryPoint()
            left_temp_hand_joints = np.hstack((self.present_left_thumb_joints, self.present_left_finger_joints[4:]))
            left_traj_point.positions = left_temp_hand_joints.tolist()
            left_traj_point.time_from_start.sec = 0
            left_traj_point.time_from_start.nanosec = 0
            left_msg.points.append(left_traj_point)
            # self.left_hand_publisher_.publish(left_msg)

            # print(left_temp_hand_joints)
            # self.compare_arrays_smaller(self.toggle_pose_left, left_temp_hand_joints, self.toggle_threshold,True)
            # print(self.toggle_control_left)

            if self.toggle_control_both:
                self.left_hand_publisher_.publish(left_msg)
                # Teleoperation ON
                if self.compare_arrays_smaller(self.toggle_pose_left, left_temp_hand_joints, self.toggle_threshold) and not self.toggle_count_left and self.toggle_release_left:
                    self.toggle_count_left = True
                    self.toggle_t0_left = time.perf_counter()
                elif self.compare_arrays_smaller(self.toggle_pose_left, left_temp_hand_joints, self.toggle_threshold) and self.toggle_count_left and self.toggle_release_left:
                    self.toggle_t1_left = time.perf_counter()
                    if self.toggle_t1_left - self.toggle_t0_left > self.toggle_time:
                        self.toggle_control_left = False
                elif self.compare_arrays_greater(self.toggle_pose_left, left_temp_hand_joints, 1.0) and not self.toggle_release_left:
                    self.toggle_release_left = True
                    self.toggle_count_left = False
                else:
                    self.toggle_count_left = False
            else:
                # Teleoperation OFF
                if self.compare_arrays_smaller(self.toggle_pose_left, left_temp_hand_joints, self.toggle_threshold) and not self.toggle_count_left and self.toggle_release_left:
                    self.toggle_count_left = True
                    self.toggle_t0_left = time.perf_counter()
                elif self.compare_arrays_smaller(self.toggle_pose_left, left_temp_hand_joints, self.toggle_threshold) and self.toggle_count_left and self.toggle_release_left:
                    self.toggle_t1_left = time.perf_counter()
                    if self.toggle_t1_left - self.toggle_t0_left > self.toggle_time:
                        self.toggle_control_left = True
                elif self.compare_arrays_greater(self.toggle_pose_left, left_temp_hand_joints, 1.0) and not self.toggle_release_left:
                    self.toggle_release_left = True
                    self.toggle_count_left = False
                else:
                    self.toggle_count_left = False
                    self.toggle_control_left = False

        if self.right_hand_start:
            right_msg = JointTrajectory()
            right_msg.joint_names = self.right_joint_names
            right_traj_point = JointTrajectoryPoint()
            right_temp_hand_joints = np.hstack((self.present_right_thumb_joints, self.present_right_finger_joints[4:]))
            right_traj_point.positions = right_temp_hand_joints.tolist()
            right_traj_point.time_from_start.sec = 0
            right_traj_point.time_from_start.nanosec = 0
            right_msg.points.append(right_traj_point)
            # self.right_hand_publisher_.publish(right_msg)

            # print(right_temp_hand_joints)
            # self.compare_arrays_smaller(self.toggle_pose_right, right_temp_hand_joints, self.toggle_threshold,True)
            # print(self.toggle_control_right)

            if self.toggle_control_both:
                self.right_hand_publisher_.publish(right_msg)
                # Teleoperation ON
                if self.compare_arrays_smaller(self.toggle_pose_right, right_temp_hand_joints, self.toggle_threshold) and not self.toggle_count_right and self.toggle_release_right:
                    self.toggle_count_right = True
                    self.toggle_t0_right = time.perf_counter()
                elif self.compare_arrays_smaller(self.toggle_pose_right, right_temp_hand_joints, self.toggle_threshold) and self.toggle_count_right and self.toggle_release_right:
                    self.toggle_t1_right = time.perf_counter()
                    if self.toggle_t1_right - self.toggle_t0_right > self.toggle_time:
                        self.toggle_control_right = False
                elif self.compare_arrays_greater(self.toggle_pose_right, right_temp_hand_joints, 1.0) and not self.toggle_release_right:
                    self.toggle_release_right = True
                    self.toggle_count_right = False
                else:
                    self.toggle_count_right = False
            else:
                # Teleoperation OFF
                if self.compare_arrays_smaller(self.toggle_pose_right, right_temp_hand_joints, self.toggle_threshold) and not self.toggle_count_right and self.toggle_release_right:
                    self.toggle_count_right = True
                    self.toggle_t0_right = time.perf_counter()
                elif self.compare_arrays_smaller(self.toggle_pose_right, right_temp_hand_joints, self.toggle_threshold) and self.toggle_count_right and self.toggle_release_right:
                    self.toggle_t1_right = time.perf_counter()
                    if self.toggle_t1_right - self.toggle_t0_right > self.toggle_time:
                        self.toggle_control_right = True
                elif self.compare_arrays_greater(self.toggle_pose_right, right_temp_hand_joints, 1.0) and not self.toggle_release_right:
                    self.toggle_release_right = True
                    self.toggle_count_right = False
                else:
                    self.toggle_count_right = False
                    self.toggle_control_right = False

        # print(self.toggle_control_left, self.toggle_control_right)
        if (self.toggle_control_left and self.toggle_control_right) and (self.toggle_count_left and self.toggle_count_right):
            # Teleoperation ON
            self.toggle_control_both = True
            self.toggle_count_left = False
            self.toggle_count_right = False
            self.toggle_release_left = False
            self.toggle_release_right = False
            bool_msg = Bool()
            bool_msg.data = True
            self.toggle_publisher_.publish(bool_msg)
        elif (not self.toggle_control_left and not self.toggle_control_right) and self.toggle_control_both:
            # Teleoperation OFF
            self.toggle_control_both = False
            self.toggle_count_left = False
            self.toggle_count_right = False
            bool_msg = Bool()
            bool_msg.data = False
            self.toggle_publisher_.publish(bool_msg)

    def left_thumb_callback(self, msg):
        for i in range(len(self.present_left_thumb_joints)):
            idx = self.left_joint_names.index(msg.name[i])
            self.present_left_thumb_joints[idx] = msg.position[i]
        self.left_hand_start = True

    def right_thumb_callback(self, msg):
        for i in range(len(self.present_right_thumb_joints)):
            idx = self.right_joint_names.index(msg.name[i])
            self.present_right_thumb_joints[idx] = msg.position[i]
        self.right_hand_start = True

    def left_finger_callback(self, msg):
        for i in range(len(self.present_left_finger_joints)):
            idx = self.left_joint_names.index(msg.joint_names[i])
            self.present_left_finger_joints[idx] = msg.points[0].positions[i]

    def right_finger_callback(self, msg):
        for i in range(len(self.present_right_finger_joints)):
            idx = self.right_joint_names.index(msg.joint_names[i])
            self.present_right_finger_joints[idx] = msg.points[0].positions[i]

    def compare_arrays_smaller(self, arr1, arr2, threshold, print_diff=False):
        diff = arr1 - arr2
        abs_diff = np.abs(diff)
        max_abs_diff = np.max(abs_diff)
        is_smaller = max_abs_diff < threshold
        if print_diff:
            print(max_abs_diff)
        return is_smaller

    def compare_arrays_greater(self, arr1, arr2, threshold, print_diff=False):
        diff = arr1 - arr2
        abs_diff = np.abs(diff)
        max_abs_diff = np.max(abs_diff)
        is_greater = max_abs_diff > threshold
        if print_diff:
            print(max_abs_diff)
        return is_greater

def main(args=None):
    rclpy.init(args=args)

    hand_publisher = HandPublisher()

    rclpy.spin(hand_publisher)

    hand_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
