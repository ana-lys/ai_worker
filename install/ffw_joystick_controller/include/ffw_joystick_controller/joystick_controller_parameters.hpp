// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace joystick_controller {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        std::vector<std::string> joystick_sensors = {};
        std::vector<std::string> state_interfaces = {"JOYSTICK X VALUE", "JOYSTICK Y VALUE", "JOYSTICK TACT SWITCH"};
        std::vector<std::string> sensorxel_l_joy_reverse_interfaces = {};
        std::vector<std::string> sensorxel_r_joy_reverse_interfaces = {};
        double joystick_calibration_min = 0.0;
        double joystick_calibration_center = 2048.0;
        double joystick_calibration_max = 4096.0;
        std::vector<std::string> sensorxel_l_joy_controlled_joints = {};
        std::vector<std::string> sensorxel_r_joy_controlled_joints = {};
        std::string joint_states_topic = "/joint_states";
        std::string sensorxel_l_joy_joint_trajectory_topic = "~/joint_trajectory";
        std::string sensorxel_r_joy_joint_trajectory_topic = "~/joint_trajectory";
        double sensorxel_l_joy_jog_scale = 0.1;
        double sensorxel_r_joy_jog_scale = 0.1;
        double deadzone = 0.05;
        double long_press_duration = 2.0;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        double joystick_calibration_min = 0.0;
        double joystick_calibration_center = 2048.0;
        double joystick_calibration_max = 4096.0;
        double sensorxel_l_joy_jog_scale = 0.1;
        double sensorxel_r_joy_jog_scale = 0.1;
        double deadzone = 0.05;
        double long_press_duration = 2.0;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("joystick_controller"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "")
    : prefix_{prefix},
      logger_{std::move(logger)} {
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    /**
     * @brief Tries to update the parsed Params object
     * @param params_in The Params object to update
     * @return true if the Params object was updated, false if it was already up to date or the mutex could not be locked
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_update_params(Params & params_in) const {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
          return true;
        }
      }
      return false;
    }

    /**
     * @brief Tries to get the current Params object
     * @param params_in The Params object to fill with the current parameters
     * @return true if mutex can be locked, false if mutex could not be locked
     * @note The parameters are only filled, when the mutex can be locked and the params timestamp is different
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.joystick_calibration_min = params.joystick_calibration_min;
      output.joystick_calibration_center = params.joystick_calibration_center;
      output.joystick_calibration_max = params.joystick_calibration_max;
      output.sensorxel_l_joy_jog_scale = params.sensorxel_l_joy_jog_scale;
      output.sensorxel_r_joy_jog_scale = params.sensorxel_r_joy_jog_scale;
      output.deadzone = params.deadzone;
      output.long_press_duration = params.long_press_duration;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "joystick_sensors")) {
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.joystick_sensors = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "state_interfaces")) {
            updated_params.state_interfaces = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sensorxel_l_joy_reverse_interfaces")) {
            updated_params.sensorxel_l_joy_reverse_interfaces = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sensorxel_r_joy_reverse_interfaces")) {
            updated_params.sensorxel_r_joy_reverse_interfaces = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joystick_calibration_min")) {
            updated_params.joystick_calibration_min = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joystick_calibration_center")) {
            updated_params.joystick_calibration_center = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joystick_calibration_max")) {
            updated_params.joystick_calibration_max = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sensorxel_l_joy_controlled_joints")) {
            updated_params.sensorxel_l_joy_controlled_joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sensorxel_r_joy_controlled_joints")) {
            updated_params.sensorxel_r_joy_controlled_joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joint_states_topic")) {
            updated_params.joint_states_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sensorxel_l_joy_joint_trajectory_topic")) {
            updated_params.sensorxel_l_joy_joint_trajectory_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sensorxel_r_joy_joint_trajectory_topic")) {
            updated_params.sensorxel_r_joy_joint_trajectory_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sensorxel_l_joy_jog_scale")) {
            updated_params.sensorxel_l_joy_jog_scale = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sensorxel_r_joy_jog_scale")) {
            updated_params.sensorxel_r_joy_jog_scale = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "deadzone")) {
            updated_params.deadzone = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "long_press_duration")) {
            updated_params.long_press_duration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      if (user_callback_) {
         user_callback_(updated_params);
      }
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "joystick_sensors")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Joystick sensor names to read from";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.joystick_sensors);
          parameters_interface_->declare_parameter(prefix_ + "joystick_sensors", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "state_interfaces")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "State interfaces provided by the hardware for Joystick sensors";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.state_interfaces);
          parameters_interface_->declare_parameter(prefix_ + "state_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sensorxel_l_joy_reverse_interfaces")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "List of state interfaces to reverse after normalization";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.sensorxel_l_joy_reverse_interfaces);
          parameters_interface_->declare_parameter(prefix_ + "sensorxel_l_joy_reverse_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sensorxel_r_joy_reverse_interfaces")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "List of state interfaces to reverse after normalization";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.sensorxel_r_joy_reverse_interfaces);
          parameters_interface_->declare_parameter(prefix_ + "sensorxel_r_joy_reverse_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joystick_calibration_min")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Minimum ADC value (0)";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.joystick_calibration_min);
          parameters_interface_->declare_parameter(prefix_ + "joystick_calibration_min", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joystick_calibration_center")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Center ADC value (2048)";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.joystick_calibration_center);
          parameters_interface_->declare_parameter(prefix_ + "joystick_calibration_center", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joystick_calibration_max")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum ADC value (4096)";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.joystick_calibration_max);
          parameters_interface_->declare_parameter(prefix_ + "joystick_calibration_max", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sensorxel_l_joy_controlled_joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "List of joints to control with Joystick";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.sensorxel_l_joy_controlled_joints);
          parameters_interface_->declare_parameter(prefix_ + "sensorxel_l_joy_controlled_joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sensorxel_r_joy_controlled_joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "List of joints to control with Joystick";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.sensorxel_r_joy_controlled_joints);
          parameters_interface_->declare_parameter(prefix_ + "sensorxel_r_joy_controlled_joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joint_states_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Topic to subscribe for joint states";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.joint_states_topic);
          parameters_interface_->declare_parameter(prefix_ + "joint_states_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sensorxel_l_joy_joint_trajectory_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Topic to publish joint trajectory commands";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.sensorxel_l_joy_joint_trajectory_topic);
          parameters_interface_->declare_parameter(prefix_ + "sensorxel_l_joy_joint_trajectory_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sensorxel_r_joy_joint_trajectory_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Topic to publish joint trajectory commands";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.sensorxel_r_joy_joint_trajectory_topic);
          parameters_interface_->declare_parameter(prefix_ + "sensorxel_r_joy_joint_trajectory_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sensorxel_l_joy_jog_scale")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Scale factor for jog commands";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.sensorxel_l_joy_jog_scale);
          parameters_interface_->declare_parameter(prefix_ + "sensorxel_l_joy_jog_scale", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sensorxel_r_joy_jog_scale")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Scale factor for jog commands";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.sensorxel_r_joy_jog_scale);
          parameters_interface_->declare_parameter(prefix_ + "sensorxel_r_joy_jog_scale", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "deadzone")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Deadzone around center position (0.0 to 1.0)";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.deadzone);
          parameters_interface_->declare_parameter(prefix_ + "deadzone", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "long_press_duration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Duration in seconds to trigger long press event";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.long_press_duration);
          parameters_interface_->declare_parameter(prefix_ + "long_press_duration", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "joystick_sensors");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'joystick_sensors': {}", validation_result.error()));
      }
      updated_params.joystick_sensors = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "state_interfaces");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.state_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "sensorxel_l_joy_reverse_interfaces");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sensorxel_l_joy_reverse_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "sensorxel_r_joy_reverse_interfaces");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sensorxel_r_joy_reverse_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "joystick_calibration_min");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joystick_calibration_min = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "joystick_calibration_center");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joystick_calibration_center = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "joystick_calibration_max");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joystick_calibration_max = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "sensorxel_l_joy_controlled_joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sensorxel_l_joy_controlled_joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "sensorxel_r_joy_controlled_joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sensorxel_r_joy_controlled_joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "joint_states_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joint_states_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "sensorxel_l_joy_joint_trajectory_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sensorxel_l_joy_joint_trajectory_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "sensorxel_r_joy_joint_trajectory_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sensorxel_r_joy_joint_trajectory_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "sensorxel_l_joy_jog_scale");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sensorxel_l_joy_jog_scale = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "sensorxel_r_joy_jog_scale");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sensorxel_r_joy_jog_scale = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "deadzone");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.deadzone = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "long_press_duration");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.long_press_duration = param.as_double();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    using userParameterUpdateCB = std::function<void(const Params&)>;
    void setUserCallback(const userParameterUpdateCB& callback){
      user_callback_ = callback;
    }

    void clearUserCallback(){
      user_callback_ = {};
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
      userParameterUpdateCB user_callback_;

      rclcpp::Logger logger_;
      std::mutex mutable mutex_;
  };

} // namespace joystick_controller
