// Copyright 2024 ROBOTIS CO., LTD.
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

#include <joystick_controller/joystick_controller.hpp>

#include <chrono>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/helpers.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joystick_controller
{

JoystickController::JoystickController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
JoystickController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
JoystickController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & sensorxel_joy_name : sensorxel_joy_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(sensorxel_joy_name + "/" + interface_type);
    }
  }

  return config;
}

void JoystickController::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Store current joint states
  current_joint_states_ = *msg;

  // Initialize last_active_positions_ with current joint positions if not already done
  if (!has_joint_states_ && !params_.controlled_joints.empty()) {
    last_active_positions_.resize(params_.controlled_joints.size());
    for (size_t i = 0; i < params_.controlled_joints.size(); ++i) {
      const auto & joint_name = params_.controlled_joints[i];
      auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
        joint_name);
      if (it != current_joint_states_.name.end()) {
        size_t index = std::distance(current_joint_states_.name.begin(), it);
        last_active_positions_[i] = current_joint_states_.position[index];
      }
    }
  }

  has_joint_states_ = true;
}

controller_interface::return_type JoystickController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Don't publish anything until we've received joint states
  if (!has_joint_states_) {
    return controller_interface::return_type::OK;
  }

  bool any_sensorxel_joy_active = false;
  // Read sensorxel_joy values from hardware interfaces
  for (size_t i = 0; i < n_sensorxel_joys_; ++i) {
    // Read values for each interface type
    for (size_t j = 0; j < state_interface_types_.size(); ++j) {
      if (j >= joint_state_interface_.size() || i >= joint_state_interface_[j].size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid interface access: j=%zu, i=%zu", j, i);
        continue;
      }

      auto opt_value = joint_state_interface_[j][i].get().get_optional();
      if (!opt_value.has_value()) {
        RCLCPP_ERROR(get_node()->get_logger(), "No value for state interface [%zu][%zu]", j, i);
        continue;
      }
      double raw_adc = opt_value.value();

      // Normalize ADC value to [-1.0, 1.0] range
      double normalized_value;
      if (raw_adc < params_.joystick_calibration_center) {
        // Below center value
        normalized_value = -(params_.joystick_calibration_center - raw_adc) /
          (params_.joystick_calibration_center - params_.joystick_calibration_min);
      } else {
        // Above center value
        normalized_value = (raw_adc - params_.joystick_calibration_center) /
          (params_.joystick_calibration_max - params_.joystick_calibration_center);
      }

      // Apply deadzone
      if (std::abs(normalized_value) < params_.deadzone) {
        normalized_value = 0.0;
      } else {
        any_sensorxel_joy_active = true;
        // Scale the value to account for deadzone
        if (normalized_value > 0) {
          normalized_value = (normalized_value - params_.deadzone) / (1.0 - params_.deadzone);
        } else {
          normalized_value = (normalized_value + params_.deadzone) / (1.0 - params_.deadzone);
        }
      }

      // Check if this interface should be reversed
      const auto & interface_name = state_interface_types_[j];
      if (std::find(params_.reverse_interfaces.begin(), params_.reverse_interfaces.end(),
          interface_name) !=
        params_.reverse_interfaces.end())
      {
        normalized_value = -normalized_value;
      }

      // Store normalized value
      sensorxel_joy_values_[i][j] = normalized_value;
    }
  }

  // Handle transition from active to inactive (entering deadzone)
  if (was_active_ && !any_sensorxel_joy_active && !current_joint_states_.name.empty() &&
    !params_.controlled_joints.empty())
  {
    // Store the last active positions
    for (size_t i = 0; i < params_.controlled_joints.size(); ++i) {
      const auto & joint_name = params_.controlled_joints[i];
      auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
        joint_name);
      if (it != current_joint_states_.name.end()) {
        size_t index = std::distance(current_joint_states_.name.begin(), it);
        last_active_positions_[i] = current_joint_states_.position[index];
      }
    }
  }

  // Publish trajectory message in both active and inactive states
  if (!current_joint_states_.name.empty() && !params_.controlled_joints.empty()) {
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.header.stamp = rclcpp::Time(0);
    trajectory_msg.joint_names = params_.controlled_joints;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration(0, 0);

    if (any_sensorxel_joy_active && !sensorxel_joy_values_.empty() &&
      !sensorxel_joy_values_[0].empty())
    {
      // Calculate new positions based on sensorxel_joy values
      for (size_t i = 0; i < params_.controlled_joints.size(); ++i) {
        const auto & joint_name = params_.controlled_joints[i];
        auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
          joint_name);
        if (it != current_joint_states_.name.end()) {
          size_t index = std::distance(current_joint_states_.name.begin(), it);
          double current_position = current_joint_states_.position[index];

          // Use X value for all joints if only X interface is available
          // Otherwise use X for first joint and Y for second joint
          double sensorxel_joy_value = (state_interface_types_.size() > 1 && i == 1) ?
            sensorxel_joy_values_[0][1] : sensorxel_joy_values_[0][0];

          double new_position = current_position + sensorxel_joy_value * params_.jog_scale;
          point.positions.push_back(new_position);
          // Store the new position as last active position
          last_active_positions_[i] = new_position;
        }
      }
    } else {
      // Use stored last active positions
      point.positions = last_active_positions_;
    }

    // Set velocities and accelerations to 0
    point.velocities.resize(point.positions.size(), 0.0);
    point.accelerations.resize(point.positions.size(), 0.0);

    trajectory_msg.points.push_back(point);
    joint_trajectory_publisher_->publish(trajectory_msg);
  }

  // Publish sensorxel_joy values
  auto sensorxel_joy_msg = std_msgs::msg::Float64MultiArray();
  // Flatten the 2D vector for publishing
  for (const auto & sensorxel_joy_value : sensorxel_joy_values_) {
    sensorxel_joy_msg.data.insert(
      sensorxel_joy_msg.data.end(),
      sensorxel_joy_value.begin(),
      sensorxel_joy_value.end());
  }
  sensorxel_joy_publisher_->publish(sensorxel_joy_msg);

  // Update previous state
  was_active_ = any_sensorxel_joy_active;

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn JoystickController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // Get sensorxel_joy sensor names from parameters
  sensorxel_joy_names_ = params_.joystick_sensors;
  n_sensorxel_joys_ = sensorxel_joy_names_.size();

  if (sensorxel_joy_names_.empty()) {
    RCLCPP_WARN(logger, "'joystick_sensors' parameter is empty.");
  }

  // Initialize the sensorxel_joy values vector with the correct size
  sensorxel_joy_values_.resize(n_sensorxel_joys_);
  for (auto & vec : sensorxel_joy_values_) {
    vec.resize(state_interface_types_.size(), 0.0);
  }

  // Initialize last active positions vector
  last_active_positions_.resize(params_.controlled_joints.size(), 0.0);

  // Create publisher for sensorxel_joy values
  sensorxel_joy_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/sensorxel_joy_values", rclcpp::SystemDefaultsQoS());

  // Create publisher for joint trajectory
  joint_trajectory_publisher_ = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    params_.joint_trajectory_topic, rclcpp::SystemDefaultsQoS());

  // Create subscriber for joint states
  joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    params_.joint_states_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&JoystickController::joint_states_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "JoystickController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // Initialize state interface vector
  joint_state_interface_.resize(state_interface_types_.size());

  // Order all sensorxel_joy sensors in the storage
  for (size_t i = 0; i < state_interface_types_.size(); ++i) {
    const auto & interface = state_interface_types_[i];
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    ordered_interfaces;
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, sensorxel_joy_names_, interface, ordered_interfaces))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.",
        n_sensorxel_joys_, interface.c_str(), ordered_interfaces.size());
      return CallbackReturn::ERROR;
    }
    joint_state_interface_[i] = ordered_interfaces;
  }

  RCLCPP_INFO(get_node()->get_logger(), "JoystickController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "JoystickController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
}  // namespace joystick_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joystick_controller::JoystickController, controller_interface::ControllerInterface)
