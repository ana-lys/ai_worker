#!/usr/bin/env python3
#
# Test trajectory publisher - oscillates joint between limits
# Tests proper velocity and acceleration profiling

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class TrajectoryTest(Node):

    def __init__(self):
        super().__init__('trajectory_test')
        
        # Test on arm_l_joint1
        self.pub = self.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            10
        )
        
        # Trajectory parameters
        self.p_max = 1.5   # ±1.5 rad
        self.v_max = 2.0   # ±2.0 rad/s
        self.a_max = 4.0   # ±4.0 rad/s²
        self.dt = 0.02     # 50Hz = 0.02s
        
        # State variables
        self.p = 0.0
        self.v = 0.0
        self.a = self.a_max
        self.dir_forw = 1  # 1 = forward, 0 = backward
        
        # 50Hz control
        self.create_timer(self.dt, self.publish_trajectory)
        
        self.get_logger().info('Trajectory Test Started!')
        self.get_logger().info(f'Oscillating between ±{self.p_max} rad')
        self.get_logger().info(f'Max velocity: {self.v_max} rad/s')
        self.get_logger().info(f'Max acceleration: {self.a_max} rad/s²')

    def publish_trajectory(self):
        """Soft limit logic with continuous acceleration"""
        
        # Update position and velocity
        self.p += self.v * self.dt
        self.v += self.a * self.dt
        
        # Clamp velocity to max
        if self.v > self.v_max:
            self.v = self.v_max
        elif self.v < -self.v_max:
            self.v = -self.v_max
        
        # Soft limit logic - reverse acceleration at boundaries
        if self.p > self.p_max and self.dir_forw == 1:
            self.a = -self.a_max
            self.dir_forw = 0
            self.get_logger().info(f'Hit +limit! p={self.p:.2f}, reversing')
        elif self.p < -self.p_max and self.dir_forw == 0:
            self.a = self.a_max
            self.dir_forw = 1
            self.get_logger().info(f'Hit -limit! p={self.p:.2f}, reversing')
        
        # Publish trajectory
        traj = JointTrajectory()
        traj.joint_names = ['arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                           'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6',
                           'arm_l_joint7', 'gripper_l_joint1']
        
        point = JointTrajectoryPoint()
        point.positions = [self.p, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.velocities = [self.v, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.accelerations = [self.a, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)
        traj.points.append(point)
        
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = TrajectoryTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
