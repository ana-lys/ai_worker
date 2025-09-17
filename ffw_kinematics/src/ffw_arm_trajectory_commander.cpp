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


#include "ffw_kinematics/ffw_arm_trajectory_commander.hpp"

FfwArmTrajectoryCommander::FfwArmTrajectoryCommander()
: Node("ffw_arm_trajectory_commander"),
  left_squeeze_value_(0.0),
  right_squeeze_value_(0.0),
  has_right_ik_solution_(false),
  has_left_ik_solution_(false)
{
  this->declare_parameter<double>("trajectory_duration", 0.01);  // 10ms
  this->declare_parameter<double>("vr_squeeze_closed", 0.035);
  this->declare_parameter<double>("vr_squeeze_open", 0.095);
  this->declare_parameter<double>("gripper_pos_closed", 1.2);
  this->declare_parameter<double>("gripper_pos_open", 0.0);

  trajectory_duration_ = this->get_parameter("trajectory_duration").as_double();

  vr_squeeze_closed_ = this->get_parameter("vr_squeeze_closed").as_double();
  vr_squeeze_open_ = this->get_parameter("vr_squeeze_open").as_double();
  gripper_pos_closed_ = this->get_parameter("gripper_pos_closed").as_double();
  gripper_pos_open_ = this->get_parameter("gripper_pos_open").as_double();

  RCLCPP_INFO(this->get_logger(), "Trajectory duration: %.3f seconds", trajectory_duration_);

  right_ik_solution_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/right_arm_ik_solution", 10,
    std::bind(&FfwArmTrajectoryCommander::rightIKSolutionCallback,
    this,
    std::placeholders::_1));

  left_ik_solution_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/left_arm_ik_solution", 10,
    std::bind(&FfwArmTrajectoryCommander::leftIKSolutionCallback,
    this,
    std::placeholders::_1));

  left_squeeze_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/vr_hand/left_squeeze", 10,
    std::bind(&FfwArmTrajectoryCommander::leftSqueezeCallback,
    this,
    std::placeholders::_1));

  right_squeeze_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/vr_hand/right_squeeze", 10,
    std::bind(&FfwArmTrajectoryCommander::rightSqueezeCallback,
    this,
    std::placeholders::_1));

  right_joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

  left_joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory", 10);

  RCLCPP_INFO(this->get_logger(), "Dual-arm trajectory commander initialized");
}

void FfwArmTrajectoryCommander::rightIKSolutionCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->joint_names.empty() || msg->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty right IK solution");
    return;
  }

  const auto & point = msg->points.back();

  if (msg->joint_names.size() != point.positions.size()) {
    RCLCPP_ERROR(this->get_logger(),
        "Right IK solution: joint names (%zu) and positions (%zu) size mismatch",
        msg->joint_names.size(), point.positions.size());
    return;
  }

  // Check for NaN or infinite values
  for (const auto & pos : point.positions) {
    if (!std::isfinite(pos)) {
      RCLCPP_ERROR(this->get_logger(),
        "Right IK solution contains invalid joint position: %f", pos
      );
      return;
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Received RIGHT arm IK solution with %zu joints",
    point.positions.size());

  right_ik_solution_.name = msg->joint_names;
  right_ik_solution_.position = point.positions;
  has_right_ik_solution_ = true;

  sendRightArmTrajectory();
}

void FfwArmTrajectoryCommander::leftIKSolutionCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->joint_names.empty() || msg->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty left IK solution");
    return;
  }

  // Get the latest trajectory point
  const auto & point = msg->points.back();

  // Validate joint names and positions have same size
  if (msg->joint_names.size() != point.positions.size()) {
    RCLCPP_ERROR(this->get_logger(),
      "Left IK solution: joint names (%zu) and positions (%zu) size mismatch",
                        msg->joint_names.size(), point.positions.size());
    return;
  }

  // Check for NaN or infinite values
  for (const auto & pos : point.positions) {
    if (!std::isfinite(pos)) {
      RCLCPP_ERROR(this->get_logger(), "Left IK solution contains invalid joint position: %f", pos);
      return;
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Received LEFT arm IK solution with %zu joints",
    point.positions.size());

  left_ik_solution_.name = msg->joint_names;
  left_ik_solution_.position = point.positions;
  has_left_ik_solution_ = true;

  sendLeftArmTrajectory();
}

void FfwArmTrajectoryCommander::leftSqueezeCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  left_squeeze_value_ = msg->data;

  double gripper_position = calculateGripperPosition(left_squeeze_value_);

  RCLCPP_DEBUG(this->get_logger(), "Left squeeze: %.3f → gripper: %.3f", left_squeeze_value_,
    gripper_position);

  if (has_left_ik_solution_) {
    sendLeftArmTrajectory();
  }
}

void FfwArmTrajectoryCommander::rightSqueezeCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  right_squeeze_value_ = msg->data;

  // Calculate corresponding gripper position using parameterized mapping
  double gripper_position = calculateGripperPosition(right_squeeze_value_);

  RCLCPP_DEBUG(this->get_logger(), "Right squeeze: %.3f → gripper: %.3f", right_squeeze_value_,
    gripper_position);

  // If we have a recent IK solution, update the trajectory with new gripper value
  if (has_right_ik_solution_) {
    sendRightArmTrajectory();
  }
}

void FfwArmTrajectoryCommander::sendRightArmTrajectory()
{
  if (!has_right_ik_solution_) {
    RCLCPP_WARN(this->get_logger(), "No right IK solution available");
    return;
  }

  auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  // trajectory_msg.header.stamp = this->get_clock()->now();
  trajectory_msg.header.frame_id = "base_link";

  // Set joint names (arm joints + gripper joints)
  trajectory_msg.joint_names = right_ik_solution_.name;
  trajectory_msg.joint_names.push_back("gripper_r_joint1");

  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  point.positions = right_ik_solution_.position;

  double gripper_position = calculateGripperPosition(right_squeeze_value_);
  point.positions.push_back(gripper_position);

  point.velocities.resize(point.positions.size(), 0.0);
  point.accelerations.resize(point.positions.size(), 0.0);

  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 0;

  trajectory_msg.points.push_back(point);

  right_joint_trajectory_pub_->publish(trajectory_msg);

  RCLCPP_DEBUG(this->get_logger(), "Published RIGHT arm trajectory with %zu joints",
                    trajectory_msg.joint_names.size());
}

void FfwArmTrajectoryCommander::sendLeftArmTrajectory()
{
  if (!has_left_ik_solution_) {
    RCLCPP_WARN(this->get_logger(), "No left IK solution available");
    return;
  }

  auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  // trajectory_msg.header.stamp = this->get_clock()->now();
  trajectory_msg.header.frame_id = "base_link";
  trajectory_msg.joint_names = left_ik_solution_.name;
  trajectory_msg.joint_names.push_back("gripper_l_joint1");

  auto point = trajectory_msgs::msg::JointTrajectoryPoint();

  point.positions = left_ik_solution_.position;

  double gripper_position = calculateGripperPosition(left_squeeze_value_);
  point.positions.push_back(gripper_position);
  point.velocities.resize(point.positions.size(), 0.0);
  point.accelerations.resize(point.positions.size(), 0.0);

  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 0;

  trajectory_msg.points.push_back(point);

  left_joint_trajectory_pub_->publish(trajectory_msg);

  RCLCPP_DEBUG(this->get_logger(), "Published LEFT arm trajectory with %zu joints",
                    trajectory_msg.joint_names.size());
}

double FfwArmTrajectoryCommander::calculateGripperPosition(double squeeze_value)
{
  if (squeeze_value < vr_squeeze_closed_ || squeeze_value > vr_squeeze_open_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "VR squeeze value %.3f out of expected range [%.3f, %.3f]",
                                squeeze_value, vr_squeeze_closed_, vr_squeeze_open_);
  }
  // Normalize squeeze value to [0, 1] range
  double normalized = (squeeze_value - vr_squeeze_closed_) /
    (vr_squeeze_open_ - vr_squeeze_closed_);
  normalized = std::max(0.0, std::min(1.0, normalized));

  double gripper_position = gripper_pos_closed_ -
    (normalized * (gripper_pos_closed_ - gripper_pos_open_));

  return std::max(gripper_pos_open_, std::min(gripper_pos_closed_, gripper_position));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FfwArmTrajectoryCommander>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
