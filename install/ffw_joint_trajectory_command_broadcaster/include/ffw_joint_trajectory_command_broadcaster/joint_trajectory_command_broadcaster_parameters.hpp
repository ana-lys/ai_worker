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



namespace joint_trajectory_command_broadcaster {

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
        bool use_urdf_to_filter = true;
        std::string follower_joint_states_topic = "/joint_states";
        double sync_threshold = 0.01;
        double max_error = 0.6;
        double min_error = 0.01;
        double min_delay = 0.0;
        double max_delay = 0.4;
        std::vector<std::string> left_joints = {};
        std::vector<double> left_offsets = {};
        std::vector<std::string> left_reverse_joints = {};
        std::vector<std::string> right_joints = {};
        std::vector<double> right_offsets = {};
        std::vector<std::string> right_reverse_joints = {};
        double trigger_threshold = -0.5;
        double trigger_duration = 2.0;
        double trigger_sign = -1.0;
        struct MapInterfaceToJointState {
            std::string position = "position";
        } map_interface_to_joint_state;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        bool use_urdf_to_filter = true;
        double sync_threshold = 0.01;
        double max_error = 0.6;
        double min_error = 0.01;
        double min_delay = 0.0;
        double max_delay = 0.4;
        double trigger_threshold = -0.5;
        double trigger_duration = 2.0;
        double trigger_sign = -1.0;
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
    : ParamListener(parameters_interface, rclcpp::get_logger("joint_trajectory_command_broadcaster"), prefix) {
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
      output.use_urdf_to_filter = params.use_urdf_to_filter;
      output.sync_threshold = params.sync_threshold;
      output.max_error = params.max_error;
      output.min_error = params.min_error;
      output.min_delay = params.min_delay;
      output.max_delay = params.max_delay;
      output.trigger_threshold = params.trigger_threshold;
      output.trigger_duration = params.trigger_duration;
      output.trigger_sign = params.trigger_sign;

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
        if (param.get_name() == (prefix_ + "map_interface_to_joint_state.position")) {
            updated_params.map_interface_to_joint_state.position = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "use_urdf_to_filter")) {
            updated_params.use_urdf_to_filter = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "follower_joint_states_topic")) {
            updated_params.follower_joint_states_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "sync_threshold")) {
            updated_params.sync_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "max_error")) {
            updated_params.max_error = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "min_error")) {
            updated_params.min_error = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "min_delay")) {
            updated_params.min_delay = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "max_delay")) {
            updated_params.max_delay = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "left_joints")) {
            updated_params.left_joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "left_offsets")) {
            updated_params.left_offsets = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "left_reverse_joints")) {
            updated_params.left_reverse_joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "right_joints")) {
            updated_params.right_joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "right_offsets")) {
            updated_params.right_offsets = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "right_reverse_joints")) {
            updated_params.right_reverse_joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "trigger_threshold")) {
            updated_params.trigger_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "trigger_duration")) {
            updated_params.trigger_duration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "trigger_sign")) {
            updated_params.trigger_sign = param.as_double();
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
      if (!parameters_interface_->has_parameter(prefix_ + "map_interface_to_joint_state.position")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.map_interface_to_joint_state.position);
          parameters_interface_->declare_parameter(prefix_ + "map_interface_to_joint_state.position", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "use_urdf_to_filter")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Uses the robot_description to filter the joint trajectory. If true, the broadcaster will publish the data of the joints present in the URDF alone. If false, the broadcaster will publish the data of any interface that has type position.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.use_urdf_to_filter);
          parameters_interface_->declare_parameter(prefix_ + "use_urdf_to_filter", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "follower_joint_states_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Topic name for subscribing to follower's joint states to check synchronization.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.follower_joint_states_topic);
          parameters_interface_->declare_parameter(prefix_ + "follower_joint_states_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "sync_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Threshold value (in radians) to determine if follower joints are synced with leader joints.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.sync_threshold);
          parameters_interface_->declare_parameter(prefix_ + "sync_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "max_error")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum error value (in radians) used for adaptive timing calculation. Errors above this value will use maximum delay.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.max_error);
          parameters_interface_->declare_parameter(prefix_ + "max_error", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "min_error")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Minimum error value (in radians) used for adaptive timing calculation. Errors below this value will use minimum delay.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.min_error);
          parameters_interface_->declare_parameter(prefix_ + "min_error", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "min_delay")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Minimum delay (in seconds) for adaptive timing when joints are not synced.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.min_delay);
          parameters_interface_->declare_parameter(prefix_ + "min_delay", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "max_delay")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum delay (in seconds) for adaptive timing when joints are not synced.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.max_delay);
          parameters_interface_->declare_parameter(prefix_ + "max_delay", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "left_joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Joint names for left arm group.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.left_joints);
          parameters_interface_->declare_parameter(prefix_ + "left_joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "left_offsets")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Offsets for left arm joints.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.left_offsets);
          parameters_interface_->declare_parameter(prefix_ + "left_offsets", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "left_reverse_joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Reverse joints for left arm group.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.left_reverse_joints);
          parameters_interface_->declare_parameter(prefix_ + "left_reverse_joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "right_joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Joint names for right arm group.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.right_joints);
          parameters_interface_->declare_parameter(prefix_ + "right_joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "right_offsets")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Offsets for right arm joints.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.right_offsets);
          parameters_interface_->declare_parameter(prefix_ + "right_offsets", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "right_reverse_joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Reverse joints for right arm group.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.right_reverse_joints);
          parameters_interface_->declare_parameter(prefix_ + "right_reverse_joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "trigger_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Threshold value for gripper trigger joints to activate auto mode. Both gripper triggers must exceed this value.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.trigger_threshold);
          parameters_interface_->declare_parameter(prefix_ + "trigger_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "trigger_duration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Duration (in seconds) that triggers must be held above threshold to toggle auto mode.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.trigger_duration);
          parameters_interface_->declare_parameter(prefix_ + "trigger_duration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "trigger_sign")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Sign of the trigger joints. 1.0 for positive sign, -1.0 for negative sign.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.trigger_sign);
          parameters_interface_->declare_parameter(prefix_ + "trigger_sign", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "map_interface_to_joint_state.position");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.map_interface_to_joint_state.position = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "use_urdf_to_filter");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.use_urdf_to_filter = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "follower_joint_states_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.follower_joint_states_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "sync_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.sync_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "max_error");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.max_error = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "min_error");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.min_error = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "min_delay");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.min_delay = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "max_delay");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.max_delay = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "left_joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.left_joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "left_offsets");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.left_offsets = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "left_reverse_joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.left_reverse_joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "right_joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.right_joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "right_offsets");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.right_offsets = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "right_reverse_joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.right_reverse_joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "trigger_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.trigger_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "trigger_duration");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.trigger_duration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "trigger_sign");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.trigger_sign = param.as_double();


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

} // namespace joint_trajectory_command_broadcaster
