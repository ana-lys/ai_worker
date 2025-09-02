// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Wonho Yun


#ifndef FFW_KINEMATICS__FFW_ARM_TRAJECTORY_COMMANDER_HPP_
#define FFW_KINEMATICS__FFW_ARM_TRAJECTORY_COMMANDER_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class FfwArmTrajectoryCommander : public rclcpp::Node
{
public:
  FfwArmTrajectoryCommander();

private:
  void rightIKSolutionCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void leftIKSolutionCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void leftSqueezeCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void rightSqueezeCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void sendRightArmTrajectory();
  void sendLeftArmTrajectory();
  double calculateGripperPosition(double squeeze_value);

private:
  double trajectory_duration_;

  double vr_squeeze_closed_;
  double vr_squeeze_open_;
  double gripper_pos_closed_;
  double gripper_pos_open_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_ik_solution_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_ik_solution_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_squeeze_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_squeeze_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_joint_trajectory_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_joint_trajectory_pub_;

  sensor_msgs::msg::JointState right_ik_solution_;
  sensor_msgs::msg::JointState left_ik_solution_;
  double left_squeeze_value_;
  double right_squeeze_value_;
  bool has_right_ik_solution_;
  bool has_left_ik_solution_;
};

#endif  // FFW_KINEMATICS__FFW_ARM_TRAJECTORY_COMMANDER_HPP_
