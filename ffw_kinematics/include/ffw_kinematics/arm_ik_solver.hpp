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

#ifndef FFW_KINEMATICS__ARM_IK_SOLVER_HPP_
#define FFW_KINEMATICS__ARM_IK_SOLVER_HPP_

#include <urdf/model.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


class FfwArmIKSolver : public rclcpp::Node
{
public:
  FfwArmIKSolver();

private:
  void toggleCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  void processRobotDescription(const std::string & robot_description);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void rightTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void leftTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void extractJointNames();
  void setupJointLimits(const urdf::Model & model);
  void setupHardcodedJointLimits();
  void setupUrdfJointLimits(const urdf::Model & model);
  void setupSolvers();
  void checkCurrentJointLimits();
  void solveIK(const geometry_msgs::msg::PoseStamped & target_pose, const std::string & arm);
  void publishCurrentPoses();

private:
  // bool ik_toggle_;

  std::string base_link_;
  std::string arm_base_link_;
  std::string right_end_effector_link_;
  std::string left_end_effector_link_;

  double lift_joint_x_offset_;
  double lift_joint_y_offset_;
  double lift_joint_z_offset_;

  double max_joint_step_degrees_;
  int ik_max_iterations_;
  double ik_tolerance_;
  bool use_hardcoded_joint_limits_;

  // Low-pass filter parameter (0..1). 0 = hold current, 1 = full target
  double lpf_alpha_;

  // Hybrid IK parameters
  bool use_hybrid_ik_;
  double current_position_weight_;
  double previous_solution_weight_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_pose_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_joint_solution_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_joint_solution_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_current_pose_pub_;

  rclcpp::TimerBase::SharedPtr pose_timer_;

  KDL::Chain right_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> right_fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> right_ik_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> right_ik_solver_jl_;

  KDL::Chain left_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> left_fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> left_ik_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> left_ik_solver_jl_;

  KDL::JntArray right_q_min_;
  KDL::JntArray right_q_max_;

  KDL::JntArray left_q_min_;
  KDL::JntArray left_q_max_;

  // Hardcoded joint limits (7 joints per arm)
  // Right arm limits (index 0-6: joint1-joint7)
  std::vector<float> right_min_joint_positions_ = {
    -3.14, -3.14, -1.57, -2.9361,
    -1.57, -1.57, -1.5804
  };

  std::vector<float> right_max_joint_positions_ = {
    1.57, 0.0, 1.57, 0.0,
    1.57, 1.57, 1.8201
  };

  // Left arm limits (index 0-6: joint1-joint7)
  // Joint2 and Joint7 have inverted rotation direction compared to right arm
  std::vector<float> left_min_joint_positions_ = {
    -3.14, 0.0, -1.57, -2.9361,
    -1.57, -1.57, -1.8201
  };

  std::vector<float> left_max_joint_positions_ = {
    1.57, 3.14, 1.57, 0.0,
    1.57, 1.57, 1.5804
  };

  std::vector<std::string> right_joint_names_;
  std::vector<double> right_current_joint_positions_;

  std::vector<std::string> left_joint_names_;
  std::vector<double> left_current_joint_positions_;

  int lift_joint_index_;    // Not used in arm-only chain
  double lift_joint_position_;    // Current lift joint position for coordinate transformation

  // Previous IK solutions for hybrid approach
  KDL::JntArray right_previous_solution_;
  KDL::JntArray left_previous_solution_;
  bool setup_complete_;
  bool has_joint_states_;
  bool has_previous_solution_;
};

#endif  // FFW_KINEMATICS__ARM_IK_SOLVER_HPP_
