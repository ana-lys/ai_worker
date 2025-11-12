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


#include "ffw_kinematics/arm_ik_solver.hpp"

FfwArmIKSolver::FfwArmIKSolver()
: Node("arm_ik_solver"),
  lift_joint_index_(-1),
  setup_complete_(false),
  has_joint_states_(false),
  has_previous_solution_(false)
{
  this->declare_parameter<std::string>("base_link", "base_link");
  this->declare_parameter<std::string>("arm_base_link", "arm_base_link");
  this->declare_parameter<std::string>("right_end_effector_link", "arm_r_link7");
  this->declare_parameter<std::string>("left_end_effector_link", "arm_l_link7");
  this->declare_parameter<std::string>("right_target_pose_topic", "/vr_hand/right_wrist");
  this->declare_parameter<std::string>("left_target_pose_topic", "/vr_hand/left_wrist");

  this->declare_parameter<std::string>("right_ik_solution_topic", "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory");
  this->declare_parameter<std::string>("left_ik_solution_topic", "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory");
  this->declare_parameter<std::string>("right_current_pose_topic",
    "/right_current_end_effector_pose");
  this->declare_parameter<std::string>("left_current_pose_topic",
    "/left_current_end_effector_pose");

  // Coordinate transformation parameters (lift_joint origin from URDF)
  this->declare_parameter<double>("lift_joint_x_offset", 0.0055);
  this->declare_parameter<double>("lift_joint_y_offset", 0.0);
  this->declare_parameter<double>("lift_joint_z_offset", 1.6316);

  // IK solver parameters
  this->declare_parameter<double>("max_joint_step_degrees", 50.0);
  this->declare_parameter<int>("ik_max_iterations", 800);
  this->declare_parameter<double>("ik_tolerance", 1e-2);

  // Hybrid IK parameters
  this->declare_parameter<bool>("use_hybrid_ik", true);
  this->declare_parameter<double>("current_position_weight", 0.5);  // Weight for current robot position
  this->declare_parameter<double>("previous_solution_weight", 0.5); // Weight for previous IK solution

  // Low-pass filter between current state and IK target
  this->declare_parameter<double>("lpf_alpha", 0.9);

  // Joint limits parameters (can be overridden if needed)
  this->declare_parameter<bool>("use_hardcoded_joint_limits", true);

  // Adaptive delay parameters (for smooth movement when IK suddenly solves)
  this->declare_parameter<double>("min_error", 0.05);  // rad - below this, no delay (fast)
  this->declare_parameter<double>("max_error", 0.8);   // rad - above this, max delay (slow)
  this->declare_parameter<double>("min_delay", 0.0);   // sec - minimum delay (fast)
  this->declare_parameter<double>("max_delay", 0.3);   // sec - maximum delay (slow)
  this->declare_parameter<double>("target_change_threshold", 1.5);  // rad - threshold for sudden IK solve
  this->declare_parameter<double>("target_change_max_delay", 0.8);  // sec - max delay when sudden change detected

  // Slow start parameters (only at pedal press start)
  this->declare_parameter<double>("slow_start_duration", 3.0);  // sec - duration of slow start after first message
  this->declare_parameter<double>("slow_start_delay", 0.5);    // sec - delay during slow start period
  this->declare_parameter<double>("slow_start_sync_threshold", 0.15);  // rad - error threshold for early sync exit

  // Permanent sync threshold (similar to joint_trajectory_command_broadcaster)
  this->declare_parameter<double>("sync_threshold", 0.01);  // rad - threshold for permanent sync state

  base_link_ = this->get_parameter("base_link").as_string();
  arm_base_link_ = this->get_parameter("arm_base_link").as_string();
  right_end_effector_link_ = this->get_parameter("right_end_effector_link").as_string();
  left_end_effector_link_ = this->get_parameter("left_end_effector_link").as_string();
  std::string right_target_pose_topic = this->get_parameter("right_target_pose_topic").as_string();
  std::string left_target_pose_topic = this->get_parameter("left_target_pose_topic").as_string();

  std::string right_ik_solution_topic = this->get_parameter("right_ik_solution_topic").as_string();
  std::string left_ik_solution_topic = this->get_parameter("left_ik_solution_topic").as_string();
  std::string right_current_pose_topic =
    this->get_parameter("right_current_pose_topic").as_string();
  std::string left_current_pose_topic = this->get_parameter("left_current_pose_topic").as_string();

  lift_joint_x_offset_ = this->get_parameter("lift_joint_x_offset").as_double();
  lift_joint_y_offset_ = this->get_parameter("lift_joint_y_offset").as_double();
  lift_joint_z_offset_ = this->get_parameter("lift_joint_z_offset").as_double();

  max_joint_step_degrees_ = this->get_parameter("max_joint_step_degrees").as_double();
  ik_max_iterations_ = this->get_parameter("ik_max_iterations").as_int();
  ik_tolerance_ = this->get_parameter("ik_tolerance").as_double();

  use_hybrid_ik_ = this->get_parameter("use_hybrid_ik").as_bool();
  current_position_weight_ = this->get_parameter("current_position_weight").as_double();
  previous_solution_weight_ = this->get_parameter("previous_solution_weight").as_double();

  lpf_alpha_ = this->get_parameter("lpf_alpha").as_double();
  if (lpf_alpha_ < 0.0) { lpf_alpha_ = 0.0; }
  if (lpf_alpha_ > 1.0) { lpf_alpha_ = 1.0; }

  use_hardcoded_joint_limits_ = this->get_parameter("use_hardcoded_joint_limits").as_bool();

  // Get adaptive delay parameters
  min_error_ = this->get_parameter("min_error").as_double();
  max_error_ = this->get_parameter("max_error").as_double();
  min_delay_ = this->get_parameter("min_delay").as_double();
  max_delay_ = this->get_parameter("max_delay").as_double();
  target_change_threshold_ = this->get_parameter("target_change_threshold").as_double();
  target_change_max_delay_ = this->get_parameter("target_change_max_delay").as_double();

  // Get slow start parameters
  slow_start_duration_ = this->get_parameter("slow_start_duration").as_double();
  slow_start_delay_ = this->get_parameter("slow_start_delay").as_double();
  slow_start_sync_threshold_ = this->get_parameter("slow_start_sync_threshold").as_double();

  // Get permanent sync threshold
  sync_threshold_ = this->get_parameter("sync_threshold").as_double();

  // Initialize previous target positions
  right_previous_target_positions_.clear();
  left_previous_target_positions_.clear();

  // Initialize slow start tracking (triggered by VR toggle false -> true transition)
  vr_toggle_state_ = false;
  right_soft_start_active_ = false;
  left_soft_start_active_ = false;
  right_slow_start_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  left_slow_start_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Initialize permanent sync state tracking
  right_joints_synced_ = false;
  left_joints_synced_ = false;
  right_first_publish_ = true;
  left_first_publish_ = true;

  RCLCPP_INFO(this->get_logger(), "🚀 Dual-Arm IK Solver starting...");
  RCLCPP_INFO(this->get_logger(), "Base link: %s", base_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "Arm base link: %s", arm_base_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "Right end effector link: %s", right_end_effector_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "Left end effector link: %s", left_end_effector_link_.c_str());

  robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local(),
    std::bind(&FfwArmIKSolver::robotDescriptionCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&FfwArmIKSolver::jointStateCallback, this, std::placeholders::_1));

  vr_toggle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/vr_control/toggle", 10,
    std::bind(&FfwArmIKSolver::vrToggleCallback, this, std::placeholders::_1));

  right_target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    right_target_pose_topic, 10,
    std::bind(&FfwArmIKSolver::rightTargetPoseCallback, this, std::placeholders::_1));

  left_target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    left_target_pose_topic, 10,
    std::bind(&FfwArmIKSolver::leftTargetPoseCallback, this, std::placeholders::_1));

  right_joint_solution_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    right_ik_solution_topic, 10);

  left_joint_solution_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    left_ik_solution_topic, 10);

  right_current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    right_current_pose_topic, 10);

  left_current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    left_current_pose_topic, 10);

  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this,
    "/robot_state_publisher");

  if (param_client->wait_for_service(std::chrono::seconds(2))) {
    try {
      auto parameters = param_client->get_parameters({"robot_description"});
      if (!parameters.empty() &&
        parameters[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        std::string robot_desc = parameters[0].as_string();
        if (!robot_desc.empty()) {
          RCLCPP_INFO(this->get_logger(), "Retrieved robot_description from parameter server");
          processRobotDescription(robot_desc);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to get robot_description from parameter: %s",
        e.what());
      RCLCPP_INFO(this->get_logger(), "Waiting for robot_description topic...");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Waiting for robot_description topic...");
  }

  RCLCPP_INFO(this->get_logger(),
    "✅ Dual-arm IK solver initialized. Waiting for target poses on:");
  RCLCPP_INFO(this->get_logger(), "Right arm: %s", right_target_pose_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Left arm: %s", left_target_pose_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing IK solutions on:");
  RCLCPP_INFO(this->get_logger(), "Right arm: %s", right_ik_solution_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Left arm: %s", left_ik_solution_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing current poses on:");
  RCLCPP_INFO(this->get_logger(), "Right arm: %s", right_current_pose_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Left arm: %s", left_current_pose_topic.c_str());
}

void FfwArmIKSolver::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received robot_description via topic");
  processRobotDescription(msg->data);
}

void FfwArmIKSolver::processRobotDescription(const std::string & robot_description)
{
  RCLCPP_INFO(this->get_logger(), "Processing robot_description (%zu bytes)",
    robot_description.size());

  try {
    // Parse URDF
    urdf::Model model;
    if (!model.initString(robot_description)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
      return;
    }

    // Build KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_description, kdl_tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
      return;
    }

    // Extract right arm chain
    if (!kdl_tree.getChain(arm_base_link_, right_end_effector_link_, right_chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get right arm chain from %s to %s",
                           arm_base_link_.c_str(), right_end_effector_link_.c_str());
      return;
    }

    // Extract left arm chain
    if (!kdl_tree.getChain(arm_base_link_, left_end_effector_link_, left_chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get left arm chain from %s to %s",
                           arm_base_link_.c_str(), left_end_effector_link_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "✅ KDL chains extracted successfully:");
    RCLCPP_INFO(this->get_logger(), "   Right arm: %d joints, %d segments",
                       right_chain_.getNrOfJoints(), right_chain_.getNrOfSegments());
    RCLCPP_INFO(this->get_logger(), "   Left arm: %d joints, %d segments",
                       left_chain_.getNrOfJoints(), left_chain_.getNrOfSegments());

    // Extract joint names
    extractJointNames();

    // Setup joint limits
    setupJointLimits(model);

    // Create solvers for both arms
    setupSolvers();

    // Initialize previous solution arrays
    right_previous_solution_.resize(right_chain_.getNrOfJoints());
    left_previous_solution_.resize(left_chain_.getNrOfJoints());

    // Initialize with zero positions
    for (unsigned int i = 0; i < right_chain_.getNrOfJoints(); i++) {
      right_previous_solution_(i) = 0.0;
    }
    for (unsigned int i = 0; i < left_chain_.getNrOfJoints(); i++) {
      left_previous_solution_(i) = 0.0;
    }

    setup_complete_ = true;
    RCLCPP_INFO(this->get_logger(), "🎉 IK solver setup complete!");
    RCLCPP_INFO(this->get_logger(), "   Hybrid IK: %s (current: %.1f%%, previous: %.1f%%)",
                use_hybrid_ik_ ? "enabled" : "disabled",
                current_position_weight_ * 100.0, previous_solution_weight_ * 100.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during robot description processing: %s", e.what());
  }
}

void FfwArmIKSolver::extractJointNames()
{
  // Extract right arm joint names
  right_joint_names_.clear();
  for (unsigned int i = 0; i < right_chain_.getNrOfSegments(); i++) {
    const KDL::Segment & segment = right_chain_.getSegment(i);
    if (segment.getJoint().getType() != KDL::Joint::None) {
      right_joint_names_.push_back(segment.getJoint().getName());
    }
  }

  // Extract left arm joint names
  left_joint_names_.clear();
  for (unsigned int i = 0; i < left_chain_.getNrOfSegments(); i++) {
    const KDL::Segment & segment = left_chain_.getSegment(i);
    if (segment.getJoint().getType() != KDL::Joint::None) {
      left_joint_names_.push_back(segment.getJoint().getName());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Right arm joint names extracted:");
  for (size_t i = 0; i < right_joint_names_.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, right_joint_names_[i].c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Left arm joint names extracted:");
  for (size_t i = 0; i < left_joint_names_.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, left_joint_names_[i].c_str());
  }
}

void FfwArmIKSolver::setupJointLimits(const urdf::Model & model)
{
  if (use_hardcoded_joint_limits_) {
    setupHardcodedJointLimits();
  } else {
    setupUrdfJointLimits(model);
  }
}

void FfwArmIKSolver::setupHardcodedJointLimits()
{
  // Setup right arm joint limits using hardcoded values
  unsigned int right_num_joints = right_chain_.getNrOfJoints();
  right_q_min_.resize(right_num_joints);
  right_q_max_.resize(right_num_joints);

  RCLCPP_INFO(this->get_logger(), "🔒 Setting up right arm joint limits with hardcoded values:");

  // Check if we have the expected number of joints
  if (right_num_joints != right_min_joint_positions_.size()) {
    RCLCPP_WARN(this->get_logger(),
      "Right arm joint count mismatch: chain has %d joints, hardcoded limits for %zu",
      right_num_joints, right_min_joint_positions_.size());
  } else {
    for (unsigned int i = 0; i < right_num_joints; i++) {
      right_q_min_(i) = right_min_joint_positions_[i];
      right_q_max_(i) = right_max_joint_positions_[i];
      RCLCPP_INFO(this->get_logger(), "  Joint %d: [%.3f, %.3f] rad",
                           i, right_q_min_(i), right_q_max_(i));
    }
  }

  // Setup left arm joint limits using hardcoded values
  unsigned int left_num_joints = left_chain_.getNrOfJoints();
  left_q_min_.resize(left_num_joints);
  left_q_max_.resize(left_num_joints);

  RCLCPP_INFO(this->get_logger(), "🔒 Setting up left arm joint limits with hardcoded values:");

  // Check if we have the expected number of joints
  if (left_num_joints != left_min_joint_positions_.size()) {
    RCLCPP_WARN(this->get_logger(),
      "Left arm joint count mismatch: chain has %d joints, hardcoded limits for %zu",
      left_num_joints, left_min_joint_positions_.size());
  } else {
    for (unsigned int i = 0; i < left_num_joints; i++) {
      left_q_min_(i) = left_min_joint_positions_[i];
      left_q_max_(i) = left_max_joint_positions_[i];
      RCLCPP_INFO(this->get_logger(), "  Joint %d: [%.3f, %.3f] rad",
      i, left_q_min_(i), left_q_max_(i));
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "✅ Joint limits configured for both arms using hardcoded values");
}

void FfwArmIKSolver::setupUrdfJointLimits(const urdf::Model & model)
{
  RCLCPP_INFO(this->get_logger(), "🔒 Setting up joint limits from URDF...");

  // Setup right arm joint limits from URDF
  unsigned int right_num_joints = right_chain_.getNrOfJoints();
  right_q_min_.resize(right_num_joints);
  right_q_max_.resize(right_num_joints);

  for (size_t i = 0; i < right_joint_names_.size() && i < right_num_joints; i++) {
    auto joint = model.getJoint(right_joint_names_[i]);
    if (joint && joint->limits) {
      right_q_min_(i) = joint->limits->lower;
      right_q_max_(i) = joint->limits->upper;
      RCLCPP_INFO(this->get_logger(), "  Right %s: [%.3f, %.3f] rad",
                           right_joint_names_[i].c_str(), right_q_min_(i), right_q_max_(i));
    } else {
      RCLCPP_WARN(this->get_logger(), "No limits found for right joint: %s",
        right_joint_names_[i].c_str());
      right_q_min_(i) = -M_PI;
      right_q_max_(i) = M_PI;
    }
  }

  unsigned int left_num_joints = left_chain_.getNrOfJoints();
  left_q_min_.resize(left_num_joints);
  left_q_max_.resize(left_num_joints);

  for (size_t i = 0; i < left_joint_names_.size() && i < left_num_joints; i++) {
    auto joint = model.getJoint(left_joint_names_[i]);
    if (joint && joint->limits) {
      left_q_min_(i) = joint->limits->lower;
      left_q_max_(i) = joint->limits->upper;
      RCLCPP_INFO(this->get_logger(), "  Left %s: [%.3f, %.3f] rad",
                           left_joint_names_[i].c_str(), left_q_min_(i), left_q_max_(i));
    } else {
      RCLCPP_WARN(this->get_logger(), "No limits found for left joint: %s",
        left_joint_names_[i].c_str());
      left_q_min_(i) = -M_PI;
      left_q_max_(i) = M_PI;
    }
  }

  RCLCPP_INFO(this->get_logger(), "✅ Joint limits configured for both arms from URDF");
}

void FfwArmIKSolver::setupSolvers()
{
  // Right arm solvers
  right_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(right_chain_);
  right_ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(right_chain_);
  right_ik_solver_jl_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
    right_chain_, right_q_min_, right_q_max_, *right_fk_solver_, *right_ik_vel_solver_,
    ik_max_iterations_, ik_tolerance_);

  // Left arm solvers
  left_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(left_chain_);
  left_ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(left_chain_);
  left_ik_solver_jl_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
    left_chain_, left_q_min_, left_q_max_, *left_fk_solver_, *left_ik_vel_solver_,
    ik_max_iterations_, ik_tolerance_);

  RCLCPP_INFO(this->get_logger(), "✅ IK solvers created for both arms");
  RCLCPP_INFO(this->get_logger(), "   Max iterations: %d, Tolerance: %.2e", ik_max_iterations_,
    ik_tolerance_);
}

void FfwArmIKSolver::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!setup_complete_) {
    return;
  }

  // Extract current lift_joint position for coordinate transformation
  lift_joint_position_ = 0.0;
  for (size_t i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "lift_joint") {
      lift_joint_position_ = msg->position[i];
      break;
    }
  }

  // Extract right arm joint positions
  right_current_joint_positions_.assign(right_joint_names_.size(), 0.0);
  bool right_all_joints_found = true;
  for (size_t i = 0; i < right_joint_names_.size(); i++) {
    auto it = std::find(msg->name.begin(), msg->name.end(), right_joint_names_[i]);
    if (it != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it);
      right_current_joint_positions_[i] = msg->position[idx];
    } else {
      right_all_joints_found = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Joint %s not found in joint_states",
      right_joint_names_[i].c_str());
    }
  }

  // Extract left arm joint positions
  left_current_joint_positions_.assign(left_joint_names_.size(), 0.0);
  bool left_all_joints_found = true;
  for (size_t i = 0; i < left_joint_names_.size(); i++) {
    auto it = std::find(msg->name.begin(), msg->name.end(), left_joint_names_[i]);
    if (it != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it);
      left_current_joint_positions_[i] = msg->position[idx];
    } else {
      left_all_joints_found = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "Joint %s not found in joint_states",
        left_joint_names_[i].c_str());
    }
  }

  if (right_all_joints_found && left_all_joints_found && !has_joint_states_) {
    has_joint_states_ = true;
    RCLCPP_INFO(this->get_logger(), "✅ All joint states received. IK solver ready!");
    checkCurrentJointLimits();
  }

  // Publish current poses on each joint state update
  publishCurrentPoses();
}

void FfwArmIKSolver::checkCurrentJointLimits()
{
  RCLCPP_INFO(this->get_logger(), "🔍 Checking current joint positions against limits:");

  // Check right arm joints
  bool right_all_within_limits = true;
  RCLCPP_INFO(this->get_logger(), "Right arm joints:");
  for (size_t i = 0; i < right_current_joint_positions_.size(); i++) {
    double pos = right_current_joint_positions_[i];
    double min_limit = right_q_min_(i);
    double max_limit = right_q_max_(i);
    bool within_limits = (pos >= min_limit && pos <= max_limit);
    if (!within_limits) {right_all_within_limits = false;}

    RCLCPP_INFO(this->get_logger(), "  %s: %.3f rad [%.3f, %.3f] %s",
                       right_joint_names_[i].c_str(), pos, min_limit, max_limit,
                       within_limits ? "✅" : "❌");
  }

  // Check left arm joints
  bool left_all_within_limits = true;
  RCLCPP_INFO(this->get_logger(), "Left arm joints:");
  for (size_t i = 0; i < left_current_joint_positions_.size(); i++) {
    double pos = left_current_joint_positions_[i];
    double min_limit = left_q_min_(i);
    double max_limit = left_q_max_(i);
    bool within_limits = (pos >= min_limit && pos <= max_limit);
    if (!within_limits) {left_all_within_limits = false;}

    RCLCPP_INFO(this->get_logger(), "  %s: %.3f rad [%.3f, %.3f] %s",
                       left_joint_names_[i].c_str(), pos, min_limit, max_limit,
                       within_limits ? "✅" : "❌");
  }

  if (right_all_within_limits && left_all_within_limits) {
    RCLCPP_INFO(this->get_logger(), "✅ All current joint positions are within limits");
  } else {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Some joints are outside limits - this is OK for initialization");
  }
}

void FfwArmIKSolver::vrToggleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool new_state = msg->data;

  // Detect false to true transition - activate soft start
  if (new_state && !vr_toggle_state_) {
    right_soft_start_active_ = true;
    left_soft_start_active_ = true;
    rclcpp::Time now = this->get_clock()->now();
    right_slow_start_end_time_ = now + rclcpp::Duration::from_seconds(slow_start_duration_);
    left_slow_start_end_time_ = now + rclcpp::Duration::from_seconds(slow_start_duration_);
    // Reset sync state when starting slow start (similar to broadcaster)
    right_joints_synced_ = false;
    left_joints_synced_ = false;
    right_first_publish_ = true;
    left_first_publish_ = true;
    RCLCPP_INFO(this->get_logger(),
                "🚀 [ARM IK] VR toggle: false -> true, slow start ACTIVATED");
    RCLCPP_INFO(this->get_logger(),
                "   Duration: %.1f sec, Delay: %.3f sec (both arms)",
                slow_start_duration_, slow_start_delay_);
  }

  // Reset soft start when toggle becomes false
  if (!new_state && vr_toggle_state_) {
    right_soft_start_active_ = false;
    left_soft_start_active_ = false;
    RCLCPP_INFO(this->get_logger(),
                "⏹️  [ARM IK] VR toggle: true -> false, slow start DEACTIVATED");
  }

  vr_toggle_state_ = new_state;
}

void FfwArmIKSolver::rightTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!setup_complete_ || !has_joint_states_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "IK solver not ready. Ignoring right target pose.");
    return;
  }

        // Validate pose input
  if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) ||
    !std::isfinite(msg->pose.position.z) || !std::isfinite(msg->pose.orientation.w) ||
    !std::isfinite(msg->pose.orientation.x) || !std::isfinite(msg->pose.orientation.y) ||
    !std::isfinite(msg->pose.orientation.z))
  {
    RCLCPP_ERROR(this->get_logger(), "Received invalid (non-finite) right target pose. Ignoring.");
    return;
  }

  // RCLCPP_INFO(this->get_logger(), "Received RIGHT arm target pose (base_link frame):");
  // RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
  //   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // Transform pose from base_link to arm_base_link frame
  geometry_msgs::msg::PoseStamped arm_base_pose = *msg;

  // Transform: base_link -> arm_base_link using configured offsets
  arm_base_pose.pose.position.x -= lift_joint_x_offset_;
  arm_base_pose.pose.position.y -= lift_joint_y_offset_;
  arm_base_pose.pose.position.z -= (lift_joint_z_offset_ + lift_joint_position_);

  // RCLCPP_INFO(this->get_logger(), "🔄 Transformed to arm_base_link frame (RIGHT):");
  // RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f (lift: %.3f m)",
  //                  arm_base_pose.pose.position.x, arm_base_pose.pose.position.y,
  //                  arm_base_pose.pose.position.z, lift_joint_position_);

  // Solve IK for the transformed target
  solveIK(arm_base_pose, "right");
}

void FfwArmIKSolver::leftTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!setup_complete_ || !has_joint_states_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "IK solver not ready. Ignoring left target pose.");
    return;
  }

        // Validate pose input
  if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) ||
    !std::isfinite(msg->pose.position.z) || !std::isfinite(msg->pose.orientation.w) ||
    !std::isfinite(msg->pose.orientation.x) || !std::isfinite(msg->pose.orientation.y) ||
    !std::isfinite(msg->pose.orientation.z))
  {
    RCLCPP_ERROR(this->get_logger(), "Received invalid (non-finite) left target pose. Ignoring.");
    return;
  }

  // RCLCPP_INFO(this->get_logger(), "🎯 Received LEFT arm target pose (base_link frame):");
  // RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f",
  //                  msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // Transform pose from base_link to arm_base_link frame
  geometry_msgs::msg::PoseStamped arm_base_pose = *msg;

  // Transform: base_link -> arm_base_link using configured offsets
  arm_base_pose.pose.position.x -= lift_joint_x_offset_;
  arm_base_pose.pose.position.y -= lift_joint_y_offset_;
  arm_base_pose.pose.position.z -= (lift_joint_z_offset_ + lift_joint_position_);

  // RCLCPP_INFO(this->get_logger(), "🔄 Transformed to arm_base_link frame (LEFT):");
  // RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f (lift: %.3f m)",
  //                  arm_base_pose.pose.position.x, arm_base_pose.pose.position.y,
  //                  arm_base_pose.pose.position.z, lift_joint_position_);

  // Solve IK for the transformed target
  solveIK(arm_base_pose, "left");
}
void FfwArmIKSolver::solveIK(
  const geometry_msgs::msg::PoseStamped & target_pose,
  const std::string & arm)
{
  // RCLCPP_INFO(this->get_logger(), "🔧 Solving %s arm IK with Joint Limits...", arm.c_str());

  // Convert target pose to KDL Frame
  KDL::Frame target_frame;
  target_frame.p.x(target_pose.pose.position.x);
  target_frame.p.y(target_pose.pose.position.y);
  target_frame.p.z(target_pose.pose.position.z);

  // Convert quaternion to rotation matrix
  KDL::Rotation rot = KDL::Rotation::Quaternion(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
  );
  target_frame.M = rot;

  // Select arm-specific variables
  KDL::Chain * chain_ptr;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> * ik_solver_ptr;
  std::vector<std::string> * joint_names_ptr;
  std::vector<double> * current_positions_ptr;
  KDL::JntArray * q_min_ptr;
  KDL::JntArray * q_max_ptr;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr * publisher_ptr;

  if (arm == "right") {
    chain_ptr = &right_chain_;
    ik_solver_ptr = &right_ik_solver_jl_;
    joint_names_ptr = &right_joint_names_;
    current_positions_ptr = &right_current_joint_positions_;
    q_min_ptr = &right_q_min_;
    q_max_ptr = &right_q_max_;
    publisher_ptr = &right_joint_solution_pub_;
  } else {
    chain_ptr = &left_chain_;
    ik_solver_ptr = &left_ik_solver_jl_;
    joint_names_ptr = &left_joint_names_;
    current_positions_ptr = &left_current_joint_positions_;
    q_min_ptr = &left_q_min_;
    q_max_ptr = &left_q_max_;
    publisher_ptr = &left_joint_solution_pub_;
  }

  // Get initial guess using hybrid approach
  KDL::JntArray q_init(chain_ptr->getNrOfJoints());
  KDL::JntArray * previous_solution_ptr = (arm == "right") ? &right_previous_solution_ : &left_previous_solution_;

  if (use_hybrid_ik_ && has_previous_solution_) {
    // Hybrid: weighted combination of current position and previous solution
    for (size_t i = 0; i < current_positions_ptr->size(); i++) {
      q_init(i) = current_position_weight_ * (*current_positions_ptr)[i] +
                  previous_solution_weight_ * (*previous_solution_ptr)(i);
    }
    RCLCPP_DEBUG(this->get_logger(), "Using hybrid initial guess for %s arm (%.1f%% current + %.1f%% previous)",
                arm.c_str(), current_position_weight_ * 100.0, previous_solution_weight_ * 100.0);
  } else {
    // Fallback: use only current positions
    for (size_t i = 0; i < current_positions_ptr->size(); i++) {
      q_init(i) = (*current_positions_ptr)[i];
    }
    RCLCPP_DEBUG(this->get_logger(), "Using current position as initial guess for %s arm", arm.c_str());
  }

  // Clamp initial guess to joint limits with margin
  const double clamp_margin = 0.1; // radians
  for (unsigned int i = 0; i < q_init.rows(); i++) {
    const double min_limit = (*q_min_ptr)(i);
    const double max_limit = (*q_max_ptr)(i);
    if (q_init(i) < min_limit) {
      double target = min_limit + clamp_margin;
      if (target > max_limit) { target = max_limit; }
      q_init(i) = target;
      RCLCPP_DEBUG(this->get_logger(), "Clamped %s arm initial guess for joint %d to min+margin (%.3f)",
        arm.c_str(), i, q_init(i));
    }
    if (q_init(i) > max_limit) {
      double target = max_limit - clamp_margin;
      if (target < min_limit) { target = min_limit; }
      q_init(i) = target;
      RCLCPP_DEBUG(this->get_logger(), "Clamped %s arm initial guess for joint %d to max-margin (%.3f)",
        arm.c_str(), i, q_init(i));
    }
  }

  KDL::JntArray q_result(chain_ptr->getNrOfJoints());

  auto initial_start_time = this->get_clock()->now();

  int ik_result = -1;

  // Static counters for throttling IK failure logs per arm
  static int right_ik_fail_count = 0;
  static int left_ik_fail_count = 0;
  static const int LOG_THROTTLE_INTERVAL = 100; // Log every 100 failures

  while (true) {
    auto start_time = this->get_clock()->now();
    ik_result = (*ik_solver_ptr)->CartToJnt(q_init, target_frame, q_result);
    auto end_time = this->get_clock()->now();
    auto duration = end_time - start_time;
    // RCLCPP_INFO(this->get_logger(), "%s arm IK solver time: %.3f ms, %.1f Hz", arm.c_str(), duration.seconds() * 1000.0, 1.0 / duration.seconds());

    if (ik_result < 0) {
      auto error_msg = (*ik_solver_ptr)->strError(ik_result);
      auto initial_duration = end_time - initial_start_time;

      // Update counter and log only periodically
      int& fail_count = (arm == "right") ? right_ik_fail_count : left_ik_fail_count;
      fail_count++;

      if (fail_count % LOG_THROTTLE_INTERVAL == 1 || fail_count <= 3) {
        // Log first 3 failures and then every 100th failure
        RCLCPP_DEBUG(this->get_logger(),
          "%s arm IK failed (error: %d, %s) - %d failures total, (time: %.3f ms, %.1f Hz), (initial time: %.3f ms, %.1f Hz)",
          arm.c_str(), ik_result, error_msg, fail_count,
          duration.seconds() * 1000.0, 1.0 / duration.seconds(),
          initial_duration.seconds() * 1000.0, 1.0 / initial_duration.seconds());
      }

      if (initial_duration.seconds() > 0.03) {
        return;
      }
    } else{
      // Reset counter on success
      if (arm == "right") {
        right_ik_fail_count = 0;
      } else {
        left_ik_fail_count = 0;
      }
      break;
    }
  }

  // If IK failed, try with different initial guesses within limits
  // if (ik_result < 0) {
  //   RCLCPP_WARN(this->get_logger(),
  //     "%s arm IK failed with current guess (error: %d), trying alternatives...", arm.c_str(),
  //     ik_result);

  //   // Try with center of joint limits as initial guess
  //   KDL::JntArray q_center(chain_ptr->getNrOfJoints());
  //   for (unsigned int i = 0; i < chain_ptr->getNrOfJoints(); i++) {
  //     q_center(i) = ((*q_min_ptr)(i) + (*q_max_ptr)(i)) / 2.0;
  //   }
  //   ik_result = (*ik_solver_ptr)->CartToJnt(q_center, target_frame, q_result);

  //   if (ik_result < 0) {
  //     // Try with home position (zeros)
  //     KDL::JntArray q_home(chain_ptr->getNrOfJoints());
  //     for (unsigned int i = 0; i < q_home.rows(); i++) {
  //       q_home(i) = 0.0;
  //     }
  //     ik_result = (*ik_solver_ptr)->CartToJnt(q_home, target_frame, q_result);
  //   }
  // }

  if (ik_result >= 0) {
    // Verify all joints are within limits
    bool all_within_limits = true;
    for (unsigned int i = 0; i < q_result.rows(); i++) {
      if (q_result(i) < (*q_min_ptr)(i) || q_result(i) > (*q_max_ptr)(i)) {
        all_within_limits = false;
        RCLCPP_WARN(this->get_logger(),
          "%s arm joint %d solution %.3f is outside limits [%.3f, %.3f]",
                               arm.c_str(), i, q_result(i), (*q_min_ptr)(i), (*q_max_ptr)(i));
      }
    }

    if (!all_within_limits) {
      RCLCPP_ERROR(this->get_logger(), "❌ %s arm IK solution violates joint limits! Skipping.",
        arm.c_str());
      return;
    }

    // Apply low-pass filter: blend current position toward IK target
    KDL::JntArray q_filtered(chain_ptr->getNrOfJoints());
    double max_lpf_delta = 0.0;
    for (unsigned int i = 0; i < q_result.rows(); i++) {
      const double current_pos = (*current_positions_ptr)[i];
      const double target_pos = q_result(i);
      q_filtered(i) = (1.0 - lpf_alpha_) * current_pos + lpf_alpha_ * target_pos;
      double lpf_delta = std::abs(q_filtered(i) - current_pos);
      max_lpf_delta = std::max(max_lpf_delta, lpf_delta);
      // Clamp to joint hard limits
      if (q_filtered(i) < (*q_min_ptr)(i)) { q_filtered(i) = (*q_min_ptr)(i); }
      if (q_filtered(i) > (*q_max_ptr)(i)) { q_filtered(i) = (*q_max_ptr)(i); }
    }

    // Clamp joint movement to max step for safety (applied to filtered values)
    const double max_joint_step = max_joint_step_degrees_ * M_PI / 180.0;
    bool clamped = false;
    double max_clamped_delta = 0.0;
    for (unsigned int i = 0; i < q_filtered.rows(); i++) {
      double delta = q_filtered(i) - (*current_positions_ptr)[i];
      double abs_delta = std::abs(delta);
      if (abs_delta > max_joint_step) {
        clamped = true;
        if (delta > 0) {
          q_filtered(i) = (*current_positions_ptr)[i] + max_joint_step;
        } else {
          q_filtered(i) = (*current_positions_ptr)[i] - max_joint_step;
        }
        max_clamped_delta = std::max(max_clamped_delta, abs_delta);
      }
    }

    // Log movement limitations periodically
    static int movement_log_counter = 0;
    movement_log_counter++;
    if (movement_log_counter % 200 == 0) {  // Log every 200 calls (~1 sec at 200Hz)
      if (clamped) {
        RCLCPP_WARN(this->get_logger(),
          "⚠️ [%s ARM] Movement CLAMPED - max step: %.1f deg, max delta: %.4f rad (%.1f deg), LPF alpha: %.3f, max LPF delta: %.4f rad",
          arm.c_str(), max_joint_step_degrees_, max_clamped_delta, max_clamped_delta * 180.0 / M_PI, lpf_alpha_, max_lpf_delta);
      } else if (max_lpf_delta > 0.01) {
        RCLCPP_INFO(this->get_logger(),
          "📊 [%s ARM] Movement - max LPF delta: %.4f rad (%.1f deg), LPF alpha: %.3f",
          arm.c_str(), max_lpf_delta, max_lpf_delta * 180.0 / M_PI, lpf_alpha_);
      }
    }

    // RCLCPP_INFO(this->get_logger(), "✅ %s arm IK solution found!", arm.c_str());

    // Store solution for next iteration (hybrid approach)
    *previous_solution_ptr = q_filtered;
    has_previous_solution_ = true;

    // Create and publish JointTrajectory message with the solution
    auto joint_trajectory = trajectory_msgs::msg::JointTrajectory();
    // joint_trajectory.header.stamp = this->get_clock()->now();
    joint_trajectory.header.frame_id = "base_link";
    joint_trajectory.joint_names = *joint_names_ptr;

    // Create trajectory point
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions.resize(q_filtered.rows());
    point.velocities.resize(q_filtered.rows(), 0.0);
    point.accelerations.resize(q_filtered.rows(), 0.0);

    for (unsigned int i = 0; i < q_filtered.rows(); i++) {
      point.positions[i] = q_filtered(i);
    }

    // Calculate adaptive delay based on error and target changes (IK suddenly solving)
    std::vector<double> current_targets(q_filtered.rows());
    for (unsigned int i = 0; i < q_filtered.rows(); i++) {
      current_targets[i] = q_filtered(i);
    }

    std::vector<double> * previous_targets_ptr = nullptr;
    if (arm == "right") {
      previous_targets_ptr = right_previous_target_positions_.empty() ? nullptr : &right_previous_target_positions_;
      right_previous_target_positions_ = current_targets;
    } else {
      previous_targets_ptr = left_previous_target_positions_.empty() ? nullptr : &left_previous_target_positions_;
      left_previous_target_positions_ = current_targets;
    }

    double mean_error = 0.0;
    double max_target_change = 0.0;

    // Log what we're comparing (for debugging)
    static int comparison_log_counter = 0;
    comparison_log_counter++;
    if (comparison_log_counter % 500 == 0) {  // Log every 500 calls (~2.5 sec at 200Hz)
      if (current_positions_ptr && current_positions_ptr->size() == current_targets.size() &&
          current_targets.size() > 0) {
        // Show first joint comparison as example
        double first_target = current_targets[0];
        double first_current = (*current_positions_ptr)[0];
        double first_diff = std::abs(first_target - first_current);
        RCLCPP_INFO(this->get_logger(),
          "🔍 [%s ARM] Error calculation: comparing TARGET (IK solution, LPF applied) vs CURRENT (leader joint states)",
          arm.c_str());
        RCLCPP_INFO(this->get_logger(),
          "   Example joint[0]: target=%.4f rad, current=%.4f rad, diff=%.4f rad",
          first_target, first_current, first_diff);
      }
    }

    double adaptive_delay = calculateAdaptiveDelay(
      current_targets,
      current_positions_ptr,
      previous_targets_ptr,
      arm,
      mean_error,
      max_target_change
    );

    // Set time from start with adaptive delay
    int64_t delay_ns = static_cast<int64_t>(adaptive_delay * 1e9);
    point.time_from_start = rclcpp::Duration::from_nanoseconds(delay_ns);
    joint_trajectory.points.push_back(point);

    // Log adaptive delay periodically
    static int right_log_counter = 0;
    static int left_log_counter = 0;
    int& log_counter = (arm == "right") ? right_log_counter : left_log_counter;
    log_counter++;

    if (log_counter % 100 == 0) {  // Log every ~0.5 second (at 200Hz typical rate)
      // Check if permanently synced
      bool is_synced = (arm == "right") ? right_joints_synced_ : left_joints_synced_;
      if (is_synced) {
        // NOTE: mean_error here is the actual calculated error (not 0.0)
        // Check if error is still within threshold
        if (mean_error <= sync_threshold_) {
          RCLCPP_INFO(this->get_logger(),
            "⚡ [%s ARM] SYNCED - time_from_start: %.4f sec (immediate), actual_mean_error: %.4f rad (within threshold: %.4f rad), LPF: %.3f, max_step: %.1f deg",
            arm.c_str(), adaptive_delay, mean_error, sync_threshold_, lpf_alpha_, max_joint_step_degrees_);
        } else {
          RCLCPP_WARN(this->get_logger(),
            "⚡ [%s ARM] SYNCED (locked) - time_from_start: %.4f sec (immediate), actual_mean_error: %.4f rad > threshold: %.4f rad (error increased but still using immediate delay), LPF: %.3f, max_step: %.1f deg",
            arm.c_str(), adaptive_delay, mean_error, sync_threshold_, lpf_alpha_, max_joint_step_degrees_);
        }
      } else if (mean_error > min_error_ || max_target_change > target_change_threshold_) {
        RCLCPP_INFO(this->get_logger(),
          "[%s ARM] NOT SYNCED - Mean error: %.4f rad, target change: %.4f rad, adaptive delay: %.4f sec (sync_threshold: %.4f rad), LPF: %.3f, max_step: %.1f deg",
          arm.c_str(), mean_error, max_target_change, adaptive_delay, sync_threshold_, lpf_alpha_, max_joint_step_degrees_);
      }
    }

    (*publisher_ptr)->publish(joint_trajectory);

    // Log joint solution
    std::string solution_log = "🎯 " + arm + " arm joint solution: [";
    for (size_t i = 0; i < joint_names_ptr->size(); i++) {
      solution_log += (*joint_names_ptr)[i] + "=" + std::to_string(q_result(i));
      if (i < joint_names_ptr->size() - 1) {solution_log += ", ";}
    }
    solution_log += "]";
    // RCLCPP_INFO(this->get_logger(), "%s", solution_log.c_str());

  } else {
    // Provide detailed error information
    std::string error_msg;
    switch(ik_result) {
      case -1: error_msg = "Failed to converge"; break;
      case -2: error_msg = "Undefined problem"; break;
      case -3: error_msg = "Degraded solution"; break;
      default: error_msg = "Unknown error"; break;
    }
    RCLCPP_ERROR(this->get_logger(), "❌ %s arm IK failed: %s (code: %d)", arm.c_str(),
      error_msg.c_str(), ik_result);

    // Calculate distance for reachability information
    double distance = sqrt(target_pose.pose.position.x * target_pose.pose.position.x +
                                  target_pose.pose.position.y * target_pose.pose.position.y +
                                  target_pose.pose.position.z * target_pose.pose.position.z);
    RCLCPP_WARN(this->get_logger(), "📐 Target distance from arm_base: %.3f m", distance);
  }
}

void FfwArmIKSolver::publishCurrentPoses()
{
  if (!setup_complete_ || !has_joint_states_) {
    return;
  }

  KDL::JntArray right_q(right_chain_.getNrOfJoints());
  for (size_t i = 0; i < right_current_joint_positions_.size(); i++) {
    right_q(i) = right_current_joint_positions_[i];
  }

  KDL::Frame right_frame;
  if (right_fk_solver_->JntToCart(right_q, right_frame) >= 0) {
    geometry_msgs::msg::PoseStamped right_pose;
    // right_pose.header.stamp = this->get_clock()->now();
    right_pose.header.frame_id = arm_base_link_;

    right_pose.pose.position.x = right_frame.p.x();
    right_pose.pose.position.y = right_frame.p.y();
    right_pose.pose.position.z = right_frame.p.z();

    double qx, qy, qz, qw;
    right_frame.M.GetQuaternion(qx, qy, qz, qw);
    right_pose.pose.orientation.x = qx;
    right_pose.pose.orientation.y = qy;
    right_pose.pose.orientation.z = qz;
    right_pose.pose.orientation.w = qw;

    right_current_pose_pub_->publish(right_pose);
  }

  // Publish left arm current pose
  KDL::JntArray left_q(left_chain_.getNrOfJoints());
  for (size_t i = 0; i < left_current_joint_positions_.size(); i++) {
    left_q(i) = left_current_joint_positions_[i];
  }

  KDL::Frame left_frame;
  if (left_fk_solver_->JntToCart(left_q, left_frame) >= 0) {
    geometry_msgs::msg::PoseStamped left_pose;
    // left_pose.header.stamp = this->get_clock()->now();
    left_pose.header.frame_id = arm_base_link_;

    left_pose.pose.position.x = left_frame.p.x();
    left_pose.pose.position.y = left_frame.p.y();
    left_pose.pose.position.z = left_frame.p.z();

    double qx, qy, qz, qw;
    left_frame.M.GetQuaternion(qx, qy, qz, qw);
    left_pose.pose.orientation.x = qx;
    left_pose.pose.orientation.y = qy;
    left_pose.pose.orientation.z = qz;
    left_pose.pose.orientation.w = qw;

    left_current_pose_pub_->publish(left_pose);
  }
}

double FfwArmIKSolver::calculateAdaptiveDelay(
  const std::vector<double> & target_positions,
  const std::vector<double> * current_positions,
  const std::vector<double> * previous_targets,
  const std::string & arm,
  double & mean_error_out,
  double & max_target_change_out
)
{
  rclcpp::Time now = this->get_clock()->now();
  bool in_slow_start = false;
  bool * soft_start_active_ptr = nullptr;
  rclcpp::Time * slow_start_end_time_ptr = nullptr;
  bool * joints_synced_ptr = nullptr;
  bool * first_publish_ptr = nullptr;

  // Check if we're in slow start period (triggered by VR toggle false -> true)
  if (arm == "right") {
    soft_start_active_ptr = &right_soft_start_active_;
    slow_start_end_time_ptr = &right_slow_start_end_time_;
    joints_synced_ptr = &right_joints_synced_;
    first_publish_ptr = &right_first_publish_;
    in_slow_start = (right_soft_start_active_ &&
                     now < right_slow_start_end_time_);
  } else {
    soft_start_active_ptr = &left_soft_start_active_;
    slow_start_end_time_ptr = &left_slow_start_end_time_;
    joints_synced_ptr = &left_joints_synced_;
    first_publish_ptr = &left_first_publish_;
    in_slow_start = (left_soft_start_active_ &&
                     now < left_slow_start_end_time_);
  }

  // Calculate error between target and current positions (used for both sync check and adaptive delay)
  // NOTE: This compares:
  //   - target_positions: IK solver output (q_filtered) - what we want leader to be at
  //   - current_positions: Leader's actual current joint positions (from /joint_states)
  // This is NOT comparing with follower positions - that happens in joint_trajectory_command_broadcaster
  std::vector<double> diffs;
  if (current_positions && current_positions->size() == target_positions.size()) {
    for (size_t i = 0; i < target_positions.size(); i++) {
      diffs.push_back(std::abs(target_positions[i] - (*current_positions)[i]));
    }
  }

  double mean_error = 0.0;
  if (!diffs.empty()) {
    double sum = 0.0;
    for (double diff : diffs) {
      sum += diff;
    }
    mean_error = sum / diffs.size();
  } else {
    mean_error = max_error_;  // Conservative: slow if no data
  }

  // Check if soft start should be disabled (synced or timeout)
  if (in_slow_start) {
    // Calculate remaining time
    double remaining_time = (*slow_start_end_time_ptr - now).seconds();

    // Static counters for throttling slow start status logs
    static int right_slow_start_log_counter = 0;
    static int left_slow_start_log_counter = 0;
    int& slow_start_log_counter = (arm == "right") ? right_slow_start_log_counter : left_slow_start_log_counter;
    slow_start_log_counter++;

    // Log slow start status periodically (every 50 calls ~ 0.25 sec at 200Hz)
    if (slow_start_log_counter % 50 == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "⏳ [%s ARM] Slow start ACTIVE: remaining %.2f sec, error: %.4f rad, delay: %.3f sec",
                  arm.c_str(), remaining_time, mean_error, slow_start_delay_);
    }

    // Check timeout first
    if (now >= *slow_start_end_time_ptr) {
      // Timeout - disable soft start
      *soft_start_active_ptr = false;
      in_slow_start = false;
      slow_start_log_counter = 0;  // Reset counter
      RCLCPP_INFO(this->get_logger(),
                  "⏰ [%s ARM] Slow start TIMEOUT after %.1f sec - disabled",
                  arm.c_str(), slow_start_duration_);
    } else {
      // Check if joints are synced enough to exit slow start early
      if (mean_error <= slow_start_sync_threshold_) {
        // Error is low enough to exit slow start early (but not necessarily permanently synced)
        *soft_start_active_ptr = false;
        in_slow_start = false;
        slow_start_log_counter = 0;  // Reset counter
        RCLCPP_INFO(this->get_logger(),
                    "✅ [%s ARM] Slow start disabled early (error: %.4f rad <= %.4f rad, remaining: %.2f sec) - switching to adaptive delay",
                    arm.c_str(), mean_error, slow_start_sync_threshold_, remaining_time);
      }
    }
  }

  // If in slow start, use fixed delay (ignore error-based calculation)
  if (in_slow_start) {
    mean_error_out = 0.0;
    max_target_change_out = 0.0;
    return slow_start_delay_;
  }

  // Update sync status and handle first publish (similar to broadcaster)
  if (*first_publish_ptr) {
    *joints_synced_ptr = false;
    *first_publish_ptr = false;
    RCLCPP_INFO(this->get_logger(),
                "First publish [%s ARM] - using adaptive time_from_start based on error",
                arm.c_str());
  } else {
    // Once synced, stay synced permanently
    bool current_synced = (mean_error <= sync_threshold_);
    if (!(*joints_synced_ptr) && current_synced) {
      *joints_synced_ptr = true;
      RCLCPP_INFO(this->get_logger(),
                  "Joints synced for the first time [%s ARM] - switching to immediate time_from_start permanently (mean_error: %.4f rad <= %.4f rad)",
                  arm.c_str(), mean_error, sync_threshold_);
    }
  }

  // If permanently synced, return immediate delay (0.0)
  // NOTE: Store actual mean_error before setting it to 0.0 for logging
  double actual_mean_error = mean_error;
  if (*joints_synced_ptr) {
    mean_error_out = 0.0;  // Return 0.0 for delay calculation, but log actual error
    max_target_change_out = 0.0;
    // Log periodically to confirm immediate delay is being used
    static int right_synced_log_counter = 0;
    static int left_synced_log_counter = 0;
    int& synced_log_counter = (arm == "right") ? right_synced_log_counter : left_synced_log_counter;
    synced_log_counter++;
    if (synced_log_counter % 100 == 0) {  // Log every 100 calls (~0.5 sec at 200Hz)
      // Check if error is still below threshold (for information)
      if (actual_mean_error <= sync_threshold_) {
        RCLCPP_INFO(this->get_logger(),
                    "⚡ [%s ARM] PERMANENTLY SYNCED - using immediate delay (0.0 sec), actual_mean_error: %.4f rad (within threshold: %.4f rad)",
                    arm.c_str(), actual_mean_error, sync_threshold_);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "⚡ [%s ARM] PERMANENTLY SYNCED (locked) - using immediate delay (0.0 sec), actual_mean_error: %.4f rad > threshold: %.4f rad (error increased but still using immediate delay)",
                    arm.c_str(), actual_mean_error, sync_threshold_);
      }
    }
    return 0.0;
  }

  if (mean_error < min_error_) {
    mean_error = 0.0;
  }

  // Calculate sudden target change (IK suddenly solving)
  double max_target_change = 0.0;
  if (previous_targets && previous_targets->size() == target_positions.size()) {
    for (size_t i = 0; i < target_positions.size(); i++) {
      double change = std::abs(target_positions[i] - (*previous_targets)[i]);
      max_target_change = std::max(max_target_change, change);
    }
  }

  // Base delay from error
  double error_ratio = std::min(mean_error / max_error_, 1.0);
  double adaptive_delay = min_delay_ + (max_delay_ - min_delay_) * error_ratio;

  // Additional delay if sudden target change detected (IK suddenly solving)
  if (max_target_change > target_change_threshold_) {
    // Large target change - IK probably just solved
    double change_ratio = std::min((max_target_change - target_change_threshold_) / target_change_threshold_, 1.0);
    double additional_delay = change_ratio * (target_change_max_delay_ - max_delay_);
    adaptive_delay = std::max(adaptive_delay, max_delay_ + additional_delay);
  }

  mean_error_out = mean_error;
  max_target_change_out = max_target_change;

  return adaptive_delay;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FfwArmIKSolver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
