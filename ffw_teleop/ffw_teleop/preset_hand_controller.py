import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HandPublisher(Node):

    def __init__(self):
        super().__init__('hand_publisher')

        self.thumb_preset_threshold = 0.0 #0.7

        self.left_preset_release = np.array([
            # 1.0, 0.7, 0.5, 0.4, # option 0
            # 1.3, 1.2, 0.5, 0.4, # option 1, thumb 0.7
            0.0, 1.57, 0.0, 0.0, # option 2, thumb 0.0
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ])

        self.left_preset_grasp = np.array([
            # 2.0, 2.0, 1.4, 0.7, # option 1
            1.5, 1.2, 0.8, 0.5, # option 2
            0.0, 1.0, 1.5, 1.2,
            0.0, 1.0, 1.5, 1.2,
            0.0, 0.8, 1.5, 1.2,
            0.0, 0.2, 1.4, 1.2
        ])

        self.right_preset_release = np.array([
            # -1.0, -0.7, 0.5, 0.4, # option 0
            # -1.3, -1.2, 0.5, 0.4, # option 1
            0.0, -1.57, 0.0, 0.0, # option 2
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ])

        self.right_preset_grasp = np.array([
            # -2.0, -2.0, 1.4, 0.7, # option 1
            -1.5, -1.2, 0.8, 0.5, # option 2
            0.0, 1.0, 1.5, 1.2,
            0.0, 1.0, 1.5, 1.2,
            0.0, 0.8, 1.5, 1.2,
            0.0, 0.2, 1.4, 1.2
        ])

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

        # self.left_trigger_subscriber_ = self.create_subscription(
        #     JointTrajectory,
        #     '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        #     self.left_trigger_callback,
        #     10
        # )

        # self.right_trigger_subscriber_ = self.create_subscription(
        #     JointTrajectory,
        #     '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
        #     self.right_trigger_callback,
        #     10
        # )

        self.trigger_subscriber_ = self.create_subscription(
            JointState,
            '/topic_based_joint_commands',
            self.trigger_callback,
            10
        )

        self.left_hand_publisher_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory', 10)
        self.right_hand_publisher_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory', 10)
        self.trigger_publisher_ = self.create_publisher(JointState, '/topic_based_joint_states', 10)

    # def left_trigger_callback(self, msg):
    #     interpolation_value = 0
    #     for i in range(len(msg.points[0].positions)):
    #         if msg.joint_names[i] == 'gripper_l_joint1':
    #             interpolation_value = self.normalize_value(msg.points[0].positions[i])
    #             break
    #     left_msg = JointTrajectory()
    #     left_msg.joint_names = self.left_joint_names
    #     left_traj_point = JointTrajectoryPoint()
    #     left_traj_point.positions = (interpolation_value*self.left_preset_grasp + (1-interpolation_value)*self.left_preset_release).tolist()
    #     left_traj_point.time_from_start.sec = 0
    #     left_traj_point.time_from_start.nanosec = 0
    #     left_msg.points.append(left_traj_point)
    #     self.left_hand_publisher_.publish(left_msg)

    # def right_trigger_callback(self, msg):
    #     interpolation_value = 0
    #     for i in range(len(msg.points[0].positions)):
    #         if msg.joint_names[i] == 'gripper_r_joint1':
    #             interpolation_value = self.normalize_value(msg.points[0].positions[i])
    #             break
    #     right_msg = JointTrajectory()
    #     right_msg.joint_names = self.right_joint_names
    #     right_traj_point = JointTrajectoryPoint()
    #     right_traj_point.positions = (interpolation_value*self.right_preset_grasp + (1-interpolation_value)*self.right_preset_release).tolist()
    #     right_traj_point.time_from_start.sec = 0
    #     right_traj_point.time_from_start.nanosec = 0
    #     right_msg.points.append(right_traj_point)
    #     self.right_hand_publisher_.publish(right_msg)

    def trigger_callback(self, msg):
        left_interpolation_value = 0
        right_interpolation_value = 0
        for i in range(len(msg.name)):
            if msg.name[i] == 'gripper_l_joint1':
                left_interpolation_value = self.normalize_value(msg.position[i])
                thumb_left_interpolation_value = self.normalize_value(left_interpolation_value, self.thumb_preset_threshold, 1.0)
            elif msg.name[i] == 'gripper_r_joint1':
                right_interpolation_value = self.normalize_value(msg.position[i])
                thumb_right_interpolation_value = self.normalize_value(right_interpolation_value, self.thumb_preset_threshold, 1.0)

        # left
        left_interpolated_trajectory = np.zeros(20)
        left_interpolated_trajectory[4:] = left_interpolation_value*self.left_preset_grasp[4:] + (1-left_interpolation_value)*self.left_preset_release[4:]
        left_interpolated_trajectory[:4] = thumb_left_interpolation_value*self.left_preset_grasp[:4] + (1-thumb_left_interpolation_value)*self.left_preset_release[:4]

        # right
        right_interpolated_trajectory = np.zeros(20)
        right_interpolated_trajectory[4:] = right_interpolation_value*self.right_preset_grasp[4:] + (1-right_interpolation_value)*self.right_preset_release[4:]
        right_interpolated_trajectory[:4] = thumb_right_interpolation_value*self.right_preset_grasp[:4] + (1-thumb_right_interpolation_value)*self.right_preset_release[:4]

        left_msg = JointTrajectory()
        left_msg.joint_names = self.left_joint_names
        left_traj_point = JointTrajectoryPoint()
        # left_traj_point.positions = (left_interpolation_value*self.left_preset_grasp + (1-left_interpolation_value)*self.left_preset_release).tolist()
        left_traj_point.positions = left_interpolated_trajectory.tolist()
        left_traj_point.time_from_start.sec = 0
        left_traj_point.time_from_start.nanosec = 0
        left_msg.points.append(left_traj_point)
        self.left_hand_publisher_.publish(left_msg)

        right_msg = JointTrajectory()
        right_msg.joint_names = self.right_joint_names
        right_traj_point = JointTrajectoryPoint()
        # right_traj_point.positions = (right_interpolation_value*self.right_preset_grasp + (1-right_interpolation_value)*self.right_preset_release).tolist()
        right_traj_point.positions = right_interpolated_trajectory.tolist()
        right_traj_point.time_from_start.sec = 0
        right_traj_point.time_from_start.nanosec = 0
        right_msg.points.append(right_traj_point)
        self.right_hand_publisher_.publish(right_msg)

        trigger_msg = JointState()
        trigger_msg.name = ['gripper_l_joint1','gripper_r_joint1']
        trigger_msg.position = [left_interpolation_value, right_interpolation_value]
        self.trigger_publisher_.publish(trigger_msg)

    def normalize_value(self, value, min_old = -0.2, max_old = 1.2):
        range_old = max_old - min_old
        normalized_value = (min(max_old, max(value, min_old)) - min_old) / range_old
        return normalized_value

def main(args=None):
    rclpy.init(args=args)

    hand_publisher = HandPublisher()

    rclpy.spin(hand_publisher)

    hand_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
