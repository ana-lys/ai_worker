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



namespace ffw_swerve_drive_controller {

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
        std::vector<std::string> steering_joint_names = {"steering_front_left_joint", "steering_front_right_joint", "steering_rear_left_joint", "steering_rear_right_joint"};
        std::vector<std::string> wheel_joint_names = {"wheel_front_left_joint", "wheel_front_right_joint", "wheel_rear_left_joint", "wheel_rear_right_joint"};
        double wheel_radius = 0.033;
        std::vector<double> module_x_offsets = {0.125, 0.125, -0.125, -0.125};
        std::vector<double> module_y_offsets = {0.125, 0.125, -0.125, -0.125};
        std::vector<double> module_angle_offsets = {2.356171, 0.785375, -2.356171, -0.785375};
        std::vector<double> module_steering_limit_lower = {-1.570796, -1.570796, -1.570796, -1.570796};
        std::vector<double> module_steering_limit_upper = {1.570796, 1.570796, 1.570796, 1.570796};
        std::vector<double> module_wheel_speed_limit_lower = {-50.0, -50.0, -50.0, -50.0};
        std::vector<double> module_wheel_speed_limit_upper = {50.0, 50.0, 50.0, 50.0};
        double steering_angular_velocity_limit = 1.0;
        double steering_alignment_angle_error_threshold = 0.1;
        double wheel_velocity_during_alignment_factor = 0.1;
        std::string cmd_vel_topic = "/cmd_vel";
        double cmd_vel_timeout = 0.5;
        std::string odom_solver_method = "svd";
        std::string odometry_data_source = "feedback";
        std::string base_frame_id = "base_link";
        std::string odom_frame_id = "odom";
        bool enable_odom_tf = true;
        std::vector<double> pose_covariance_diagonal = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
        std::vector<double> twist_covariance_diagonal = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
        int64_t velocity_rolling_window_size = 1;
        bool enable_visualization = true;
        std::string visualization_marker_topic = "swerve_visualization_markers";
        double visualization_update_time = 0.1;
        bool enabled_speed_limits = true;
        bool publish_limited_velocity = true;
        struct Linear {
            struct X {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } x;
            struct Y {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } y;
        } linear;
        struct Angular {
            struct Z {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } z;
        } angular;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        double wheel_radius = 0.033;
        double steering_angular_velocity_limit = 1.0;
        double steering_alignment_angle_error_threshold = 0.1;
        double wheel_velocity_during_alignment_factor = 0.1;
        double cmd_vel_timeout = 0.5;
        bool enable_odom_tf = true;
        int64_t velocity_rolling_window_size = 1;
        bool enable_visualization = true;
        double visualization_update_time = 0.1;
        bool enabled_speed_limits = true;
        bool publish_limited_velocity = true;
        struct Linear {
            struct X {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } x;
            struct Y {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } y;
        } linear;
        struct Angular {
            struct Z {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } z;
        } angular;
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
    : ParamListener(parameters_interface, rclcpp::get_logger("ffw_swerve_drive_controller"), prefix) {
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
      output.wheel_radius = params.wheel_radius;
      output.steering_angular_velocity_limit = params.steering_angular_velocity_limit;
      output.steering_alignment_angle_error_threshold = params.steering_alignment_angle_error_threshold;
      output.wheel_velocity_during_alignment_factor = params.wheel_velocity_during_alignment_factor;
      output.cmd_vel_timeout = params.cmd_vel_timeout;
      output.enable_odom_tf = params.enable_odom_tf;
      output.velocity_rolling_window_size = params.velocity_rolling_window_size;
      output.enable_visualization = params.enable_visualization;
      output.visualization_update_time = params.visualization_update_time;
      output.enabled_speed_limits = params.enabled_speed_limits;
      output.publish_limited_velocity = params.publish_limited_velocity;
      output.linear.x.has_velocity_limits = params.linear.x.has_velocity_limits;
      output.linear.x.has_acceleration_limits = params.linear.x.has_acceleration_limits;
      output.linear.x.has_jerk_limits = params.linear.x.has_jerk_limits;
      output.linear.x.max_velocity = params.linear.x.max_velocity;
      output.linear.x.min_velocity = params.linear.x.min_velocity;
      output.linear.x.max_acceleration = params.linear.x.max_acceleration;
      output.linear.x.min_acceleration = params.linear.x.min_acceleration;
      output.linear.x.max_jerk = params.linear.x.max_jerk;
      output.linear.x.min_jerk = params.linear.x.min_jerk;
      output.linear.y.has_velocity_limits = params.linear.y.has_velocity_limits;
      output.linear.y.has_acceleration_limits = params.linear.y.has_acceleration_limits;
      output.linear.y.has_jerk_limits = params.linear.y.has_jerk_limits;
      output.linear.y.max_velocity = params.linear.y.max_velocity;
      output.linear.y.min_velocity = params.linear.y.min_velocity;
      output.linear.y.max_acceleration = params.linear.y.max_acceleration;
      output.linear.y.min_acceleration = params.linear.y.min_acceleration;
      output.linear.y.max_jerk = params.linear.y.max_jerk;
      output.linear.y.min_jerk = params.linear.y.min_jerk;
      output.angular.z.has_velocity_limits = params.angular.z.has_velocity_limits;
      output.angular.z.has_acceleration_limits = params.angular.z.has_acceleration_limits;
      output.angular.z.has_jerk_limits = params.angular.z.has_jerk_limits;
      output.angular.z.max_velocity = params.angular.z.max_velocity;
      output.angular.z.min_velocity = params.angular.z.min_velocity;
      output.angular.z.max_acceleration = params.angular.z.max_acceleration;
      output.angular.z.min_acceleration = params.angular.z.min_acceleration;
      output.angular.z.max_jerk = params.angular.z.max_jerk;
      output.angular.z.min_jerk = params.angular.z.min_jerk;

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
        if (param.get_name() == (prefix_ + "steering_joint_names")) {
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.steering_joint_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_joint_names")) {
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.wheel_joint_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_radius")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.wheel_radius = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_x_offsets")) {
            updated_params.module_x_offsets = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_y_offsets")) {
            updated_params.module_y_offsets = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_angle_offsets")) {
            updated_params.module_angle_offsets = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_steering_limit_lower")) {
            updated_params.module_steering_limit_lower = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_steering_limit_upper")) {
            updated_params.module_steering_limit_upper = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_wheel_speed_limit_lower")) {
            updated_params.module_wheel_speed_limit_lower = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_wheel_speed_limit_upper")) {
            updated_params.module_wheel_speed_limit_upper = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_angular_velocity_limit")) {
            updated_params.steering_angular_velocity_limit = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_alignment_angle_error_threshold")) {
            updated_params.steering_alignment_angle_error_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_velocity_during_alignment_factor")) {
            updated_params.wheel_velocity_during_alignment_factor = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cmd_vel_topic")) {
            updated_params.cmd_vel_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cmd_vel_timeout")) {
            updated_params.cmd_vel_timeout = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "odom_solver_method")) {
            if(auto validation_result = one_of<std::string>(param, {"normal_equation", "qr", "svd"});
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.odom_solver_method = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "odometry_data_source")) {
            if(auto validation_result = one_of<std::string>(param, {"feedback", "command"});
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.odometry_data_source = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "base_frame_id")) {
            updated_params.base_frame_id = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "odom_frame_id")) {
            updated_params.odom_frame_id = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "enable_odom_tf")) {
            updated_params.enable_odom_tf = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "pose_covariance_diagonal")) {
            if(auto validation_result = fixed_size<double>(param, 6);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.pose_covariance_diagonal = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "twist_covariance_diagonal")) {
            if(auto validation_result = fixed_size<double>(param, 6);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.twist_covariance_diagonal = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "velocity_rolling_window_size")) {
            updated_params.velocity_rolling_window_size = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "enable_visualization")) {
            updated_params.enable_visualization = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "visualization_marker_topic")) {
            updated_params.visualization_marker_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "visualization_update_time")) {
            updated_params.visualization_update_time = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "enabled_speed_limits")) {
            updated_params.enabled_speed_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_limited_velocity")) {
            updated_params.publish_limited_velocity = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.has_velocity_limits")) {
            updated_params.linear.x.has_velocity_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.has_acceleration_limits")) {
            updated_params.linear.x.has_acceleration_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.has_jerk_limits")) {
            updated_params.linear.x.has_jerk_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.max_velocity")) {
            updated_params.linear.x.max_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.min_velocity")) {
            updated_params.linear.x.min_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.max_acceleration")) {
            updated_params.linear.x.max_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.min_acceleration")) {
            updated_params.linear.x.min_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.max_jerk")) {
            updated_params.linear.x.max_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.min_jerk")) {
            updated_params.linear.x.min_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.has_velocity_limits")) {
            updated_params.linear.y.has_velocity_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.has_acceleration_limits")) {
            updated_params.linear.y.has_acceleration_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.has_jerk_limits")) {
            updated_params.linear.y.has_jerk_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.max_velocity")) {
            updated_params.linear.y.max_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.min_velocity")) {
            updated_params.linear.y.min_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.max_acceleration")) {
            updated_params.linear.y.max_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.min_acceleration")) {
            updated_params.linear.y.min_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.max_jerk")) {
            updated_params.linear.y.max_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.min_jerk")) {
            updated_params.linear.y.min_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.has_velocity_limits")) {
            updated_params.angular.z.has_velocity_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.has_acceleration_limits")) {
            updated_params.angular.z.has_acceleration_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.has_jerk_limits")) {
            updated_params.angular.z.has_jerk_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.max_velocity")) {
            updated_params.angular.z.max_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.min_velocity")) {
            updated_params.angular.z.min_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.max_acceleration")) {
            updated_params.angular.z.max_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.min_acceleration")) {
            updated_params.angular.z.min_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.max_jerk")) {
            updated_params.angular.z.max_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.min_jerk")) {
            updated_params.angular.z.min_jerk = param.as_double();
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
      if (!parameters_interface_->has_parameter(prefix_ + "steering_joint_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Names of the steering joints. Order should correspond to other module-specific parameters";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.steering_joint_names);
          parameters_interface_->declare_parameter(prefix_ + "steering_joint_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_joint_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Names of the wheel (driving) joints. Order should correspond to steering_joint_names";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.wheel_joint_names);
          parameters_interface_->declare_parameter(prefix_ + "wheel_joint_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_radius")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Radius of the wheels in meters";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.wheel_radius);
          parameters_interface_->declare_parameter(prefix_ + "wheel_radius", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_x_offsets")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "X offsets of each module from the robot's base_link origin in meters. Order matches joint_names.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.module_x_offsets);
          parameters_interface_->declare_parameter(prefix_ + "module_x_offsets", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_y_offsets")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Y offsets of each module from the robot's base_link origin in meters. Order matches joint_names.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.module_y_offsets);
          parameters_interface_->declare_parameter(prefix_ + "module_y_offsets", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_angle_offsets")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Angle offsets for each steering module in radians. This is the angle of the wheel relative to the robot's forward direction when the steering joint encoder reads zero. Order matches joint_names.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.module_angle_offsets);
          parameters_interface_->declare_parameter(prefix_ + "module_angle_offsets", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_steering_limit_lower")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Lower position limits for steering joints in radians. Order matches joint_names";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.module_steering_limit_lower);
          parameters_interface_->declare_parameter(prefix_ + "module_steering_limit_lower", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_steering_limit_upper")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Upper position limits for steering joints in radians. Order matches joint_names";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.module_steering_limit_upper);
          parameters_interface_->declare_parameter(prefix_ + "module_steering_limit_upper", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_wheel_speed_limit_lower")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Lower velocity limits for wheel joints in rad/s. Order matches joint_names";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.module_wheel_speed_limit_lower);
          parameters_interface_->declare_parameter(prefix_ + "module_wheel_speed_limit_lower", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_wheel_speed_limit_upper")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Lower velocity limits for wheel joints in rad/s. Order matches joint_names";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.module_wheel_speed_limit_upper);
          parameters_interface_->declare_parameter(prefix_ + "module_wheel_speed_limit_upper", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_angular_velocity_limit")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum angular velocity for steering joints in rad/s.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.steering_angular_velocity_limit);
          parameters_interface_->declare_parameter(prefix_ + "steering_angular_velocity_limit", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_alignment_angle_error_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Steering angle error threshold in radians to consider module aligned for full wheel speed.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.steering_alignment_angle_error_threshold);
          parameters_interface_->declare_parameter(prefix_ + "steering_alignment_angle_error_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_velocity_during_alignment_factor")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Factor to multiply wheel velocity by when steering is aligning (0.0 to 1.0).";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.wheel_velocity_during_alignment_factor);
          parameters_interface_->declare_parameter(prefix_ + "wheel_velocity_during_alignment_factor", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cmd_vel_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Topic name for incoming velocity commands.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cmd_vel_topic);
          parameters_interface_->declare_parameter(prefix_ + "cmd_vel_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cmd_vel_timeout")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Timeout for cmd_vel subscription in seconds. If no message is received within this time, the controller will stop.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cmd_vel_timeout);
          parameters_interface_->declare_parameter(prefix_ + "cmd_vel_timeout", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "odom_solver_method")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Method for solving odometry kinematics: 'normal_equation' (manual pseudo-inverse), 'qr' (QR decomposition), or 'svd' (Singular Value Decomposition).";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.odom_solver_method);
          parameters_interface_->declare_parameter(prefix_ + "odom_solver_method", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "odometry_data_source")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Source for odometry calculation: 'feedback' (use joint states) or 'command' (use commanded joint values - open-loop).";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.odometry_data_source);
          parameters_interface_->declare_parameter(prefix_ + "odometry_data_source", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "base_frame_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Robot's base frame ID for odometry and TF.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.base_frame_id);
          parameters_interface_->declare_parameter(prefix_ + "base_frame_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "odom_frame_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Odometry frame ID for odometry and TF.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.odom_frame_id);
          parameters_interface_->declare_parameter(prefix_ + "odom_frame_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "enable_odom_tf")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable/disable publishing of odom -> base_frame TF.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.enable_odom_tf);
          parameters_interface_->declare_parameter(prefix_ + "enable_odom_tf", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "pose_covariance_diagonal")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Diagonal elements of the pose covariance matrix (x, y, z, roll, pitch, yaw).";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.pose_covariance_diagonal);
          parameters_interface_->declare_parameter(prefix_ + "pose_covariance_diagonal", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "twist_covariance_diagonal")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Diagonal elements of the twist covariance matrix (vx, vy, vz, vroll, vpitch, vyaw).";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.twist_covariance_diagonal);
          parameters_interface_->declare_parameter(prefix_ + "twist_covariance_diagonal", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "velocity_rolling_window_size")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Size of the rolling window for averaging wheel velocities in odometry. 1 means no averaging.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.velocity_rolling_window_size);
          parameters_interface_->declare_parameter(prefix_ + "velocity_rolling_window_size", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "enable_visualization")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable/disable publishing of visualization markers.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.enable_visualization);
          parameters_interface_->declare_parameter(prefix_ + "enable_visualization", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "visualization_marker_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Topic name for visualization markers.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.visualization_marker_topic);
          parameters_interface_->declare_parameter(prefix_ + "visualization_marker_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "visualization_update_time")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Size of the rolling window for averaging wheel velocities in odometry. 1 means no averaging.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.visualization_update_time);
          parameters_interface_->declare_parameter(prefix_ + "visualization_update_time", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "enabled_speed_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable/disable limits on the commanded speed.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.enabled_speed_limits);
          parameters_interface_->declare_parameter(prefix_ + "enabled_speed_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_limited_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable/disable publishing of the limited velocity.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publish_limited_velocity);
          parameters_interface_->declare_parameter(prefix_ + "publish_limited_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.has_velocity_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.has_velocity_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.has_velocity_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.has_acceleration_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.has_acceleration_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.has_acceleration_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.has_jerk_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.has_jerk_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.has_jerk_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.max_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.max_velocity);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.max_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.min_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.min_velocity);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.min_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.max_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.max_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.max_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.min_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.min_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.min_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.max_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.max_jerk);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.max_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.min_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.min_jerk);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.min_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.has_velocity_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.has_velocity_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.has_velocity_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.has_acceleration_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.has_acceleration_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.has_acceleration_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.has_jerk_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.has_jerk_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.has_jerk_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.max_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.max_velocity);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.max_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.min_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.min_velocity);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.min_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.max_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.max_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.max_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.min_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.min_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.min_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.max_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.max_jerk);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.max_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.min_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.min_jerk);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.min_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.has_velocity_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.has_velocity_limits);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.has_velocity_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.has_acceleration_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.has_acceleration_limits);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.has_acceleration_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.has_jerk_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.has_jerk_limits);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.has_jerk_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.max_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.max_velocity);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.max_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.min_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.min_velocity);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.min_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.max_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.max_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.max_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.min_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.min_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.min_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.max_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.max_jerk);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.max_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.min_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.min_jerk);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.min_jerk", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "steering_joint_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steering_joint_names': {}", validation_result.error()));
      }
      updated_params.steering_joint_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_joint_names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'wheel_joint_names': {}", validation_result.error()));
      }
      updated_params.wheel_joint_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_radius");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'wheel_radius': {}", validation_result.error()));
      }
      updated_params.wheel_radius = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "module_x_offsets");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_x_offsets = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "module_y_offsets");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_y_offsets = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "module_angle_offsets");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_angle_offsets = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "module_steering_limit_lower");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_steering_limit_lower = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "module_steering_limit_upper");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_steering_limit_upper = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "module_wheel_speed_limit_lower");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_wheel_speed_limit_lower = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "module_wheel_speed_limit_upper");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_wheel_speed_limit_upper = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "steering_angular_velocity_limit");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.steering_angular_velocity_limit = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "steering_alignment_angle_error_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.steering_alignment_angle_error_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_velocity_during_alignment_factor");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheel_velocity_during_alignment_factor = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cmd_vel_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cmd_vel_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "cmd_vel_timeout");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cmd_vel_timeout = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "odom_solver_method");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = one_of<std::string>(param, {"normal_equation", "qr", "svd"});
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'odom_solver_method': {}", validation_result.error()));
      }
      updated_params.odom_solver_method = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "odometry_data_source");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = one_of<std::string>(param, {"feedback", "command"});
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'odometry_data_source': {}", validation_result.error()));
      }
      updated_params.odometry_data_source = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "base_frame_id");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.base_frame_id = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "odom_frame_id");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.odom_frame_id = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "enable_odom_tf");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.enable_odom_tf = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "pose_covariance_diagonal");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = fixed_size<double>(param, 6);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'pose_covariance_diagonal': {}", validation_result.error()));
      }
      updated_params.pose_covariance_diagonal = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "twist_covariance_diagonal");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = fixed_size<double>(param, 6);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'twist_covariance_diagonal': {}", validation_result.error()));
      }
      updated_params.twist_covariance_diagonal = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "velocity_rolling_window_size");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.velocity_rolling_window_size = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "enable_visualization");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.enable_visualization = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "visualization_marker_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.visualization_marker_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "visualization_update_time");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.visualization_update_time = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "enabled_speed_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.enabled_speed_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "publish_limited_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publish_limited_velocity = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.has_velocity_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.has_velocity_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.has_acceleration_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.has_acceleration_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.has_jerk_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.has_jerk_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.max_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.max_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.min_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.min_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.max_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.max_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.min_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.min_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.max_jerk");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.max_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.min_jerk");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.min_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.has_velocity_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.has_velocity_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.has_acceleration_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.has_acceleration_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.has_jerk_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.has_jerk_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.max_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.max_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.min_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.min_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.max_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.max_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.min_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.min_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.max_jerk");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.max_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.min_jerk");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.min_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.has_velocity_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.has_velocity_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.has_acceleration_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.has_acceleration_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.has_jerk_limits");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.has_jerk_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.max_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.max_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.min_velocity");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.min_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.max_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.max_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.min_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.min_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.max_jerk");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.max_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.min_jerk");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.min_jerk = param.as_double();


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

} // namespace ffw_swerve_drive_controller
