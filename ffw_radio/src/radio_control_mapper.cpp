#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>

#include "ffw_ik_solver.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;

namespace
{

enum class Objective
{
  END_EFFECTOR = -1,
  POSE = 0,
  NAVIGATE = 1
};

enum class Trigger
{
  ON = -1,
  OFF = 1
};

enum class Selector
{
  SECONDARY = -1,
  PRIMARY = 1
};

enum class KillSwitch
{
  KILL = -1,
  RUN = 1
};

float apply_deadband(float value, float deadband)
{
  if (std::fabs(value) < deadband) {
    return 0.0f;
  }
  return value;
}

std::vector<int> decode_universal_switches(float clean_val, float unit_length, int n, int count)
{
  float unit_scaled = (clean_val * 100.0f) / unit_length;
  int remaining = static_cast<int>(std::round(unit_scaled));
  std::vector<int> states;
  states.reserve(static_cast<size_t>(count));

  for (int i = count - 1; i >= 0; --i) {
    int weight = static_cast<int>(std::pow(n, i));
    int s = 0;

    if (n == 3) {
      if (remaining > weight / 2) {
        s = 1;
        remaining -= weight;
      } else if (remaining < -weight / 2) {
        s = -1;
        remaining += weight;
      } else {
        s = 0;
      }
    } else {
      s = (remaining >= 0) ? 1 : -1;
      remaining -= (s * weight);
    }

    states.push_back(s);
  }

  return states;
}

}  // namespace

class RadioControlMapper : public rclcpp::Node
{
public:
  struct MjModelDeleter
  {
    void operator()(mjModel * model) const
    {
      if (model != nullptr) {
        mj_deleteModel(model);
      }
    }
  };

  struct MjDataDeleter
  {
    void operator()(mjData * data) const
    {
      if (data != nullptr) {
        mj_deleteData(data);
      }
    }
  };

  RadioControlMapper()
  : Node("radio_control_mapper")
  {
    this->declare_parameter("center_offset", 0.0);
    this->declare_parameter("deadband", 0.05);
    this->declare_parameter("max_linear_speed", 1.0);
    this->declare_parameter("max_angular_speed", 1.5);
    this->declare_parameter("max_joint_speed", 0.8);
    this->declare_parameter("pose_twitch_gain", 0.25);
    this->declare_parameter("pose_twitch_ticks", 6);
    this->declare_parameter("ee_position_step", 0.01);
    this->declare_parameter("ee_deadband", 0.05);
    this->declare_parameter("command_dt_fallback", 0.02);
    this->declare_parameter("command_dt_min", 0.005);
    this->declare_parameter("command_dt_max", 0.05);
    this->declare_parameter("joint_state_timeout_sec", 1.0);
    this->declare_parameter("resume_neutral_threshold", 0.08);
    this->declare_parameter("publish_rate_hz", 100.0);
    this->declare_parameter("joint_state_topic", "/joint_states");
    this->declare_parameter("blocked_diff_threshold", 0.15);
    this->declare_parameter("blocked_consecutive_cycles", 10);
    this->declare_parameter(
      "left_arm_topic",
      "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory");
    this->declare_parameter(
      "right_arm_topic",
      "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory");
    this->declare_parameter("head_topic", "/leader/joystick_controller_left/joint_trajectory");
    this->declare_parameter("lift_topic", "/leader/joystick_controller_right/joint_trajectory");

    // Per-joint limits (can be overridden from launch)
    this->declare_parameter<std::vector<double>>(
      "left_arm_lower_limits",
      {-3.14, -0.1, -3.14, -2.9361, -3.14, -1.57, -1.8201, 0.0});
    this->declare_parameter<std::vector<double>>(
      "left_arm_upper_limits",
      {3.14, 3.14, 3.14, 1.0786, 3.14, 1.57, 1.5804, 1.1});
    this->declare_parameter<std::vector<double>>(
      "right_arm_lower_limits",
      {-3.14, -3.14, -3.14, -2.9361, -3.14, -1.57, -1.5804, 0.0});
    this->declare_parameter<std::vector<double>>(
      "right_arm_upper_limits",
      {3.14, 0.1, 3.14, 1.0786, 3.14, 1.57, 1.8201, 1.1});
    this->declare_parameter<std::vector<double>>("head_lower_limits", {-0.2317, -0.35});
    this->declare_parameter<std::vector<double>>("head_upper_limits", {0.6951, 0.35});
    this->declare_parameter<std::vector<double>>("lift_lower_limits", {-0.5});
    this->declare_parameter<std::vector<double>>("lift_upper_limits", {0.0});

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&RadioControlMapper::joy_callback, this, _1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      this->get_parameter("joint_state_topic").as_string(),
      20,
      std::bind(&RadioControlMapper::joint_state_callback, this, _1));

    base_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    linear_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "radio/linear_velocity", 10);

    left_arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("left_arm_topic").as_string(), 10);
    right_arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("right_arm_topic").as_string(), 10);
    head_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("head_topic").as_string(), 10);
    lift_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("lift_topic").as_string(), 10);

    left_arm_joint_names_ = {
      "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
      "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1"};

    right_arm_joint_names_ = {
      "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
      "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1"};

    head_joint_names_ = {"head_joint1", "head_joint2"};
    lift_joint_names_ = {"lift_joint"};

    left_arm_cmd_pos_.assign(left_arm_joint_names_.size(), 0.0);
    right_arm_cmd_pos_.assign(right_arm_joint_names_.size(), 0.0);
    head_cmd_pos_.assign(head_joint_names_.size(), 0.0);
    lift_cmd_pos_.assign(lift_joint_names_.size(), 0.0);

    left_arm_kin_pos_.assign(left_arm_joint_names_.size(), 0.0);
    right_arm_kin_pos_.assign(right_arm_joint_names_.size(), 0.0);
    head_kin_pos_.assign(head_joint_names_.size(), 0.0);
    lift_kin_pos_.assign(lift_joint_names_.size(), 0.0);

    left_arm_lower_limits_ = this->get_parameter("left_arm_lower_limits").as_double_array();
    left_arm_upper_limits_ = this->get_parameter("left_arm_upper_limits").as_double_array();
    right_arm_lower_limits_ = this->get_parameter("right_arm_lower_limits").as_double_array();
    right_arm_upper_limits_ = this->get_parameter("right_arm_upper_limits").as_double_array();
    head_lower_limits_ = this->get_parameter("head_lower_limits").as_double_array();
    head_upper_limits_ = this->get_parameter("head_upper_limits").as_double_array();
    lift_lower_limits_ = this->get_parameter("lift_lower_limits").as_double_array();
    lift_upper_limits_ = this->get_parameter("lift_upper_limits").as_double_array();

    normalize_limit_sizes(left_arm_lower_limits_, left_arm_upper_limits_, left_arm_joint_names_.size(), "left_arm");
    normalize_limit_sizes(right_arm_lower_limits_, right_arm_upper_limits_, right_arm_joint_names_.size(), "right_arm");
    normalize_limit_sizes(head_lower_limits_, head_upper_limits_, head_joint_names_.size(), "head");
    normalize_limit_sizes(lift_lower_limits_, lift_upper_limits_, lift_joint_names_.size(), "lift");

    initialize_ik_runtime();

    command_dt_ = this->get_parameter("command_dt_fallback").as_double();

    const double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    const double clamped_rate_hz = std::max(1.0, publish_rate_hz);
    const auto timer_period = std::chrono::duration<double>(1.0 / clamped_rate_hz);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&RadioControlMapper::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Pose joint level count set to 8 (7 arm + gripper)");
    log_startup_parameters();
  }

private:
  void normalize_limit_sizes(
    std::vector<double> & lower,
    std::vector<double> & upper,
    size_t expected_size,
    const std::string & group)
  {
    if (lower.size() == expected_size && upper.size() == expected_size) {
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Limit size mismatch for %s (lower=%zu upper=%zu expected=%zu). Disabling clamp for missing entries.",
      group.c_str(),
      lower.size(),
      upper.size(),
      expected_size);

    lower.resize(expected_size, -std::numeric_limits<double>::infinity());
    upper.resize(expected_size, std::numeric_limits<double>::infinity());
  }

  static std::string vector_to_string(const std::vector<double> & values)
  {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
      oss << values[i];
      if (i + 1 < values.size()) {
        oss << ", ";
      }
    }
    oss << "]";
    return oss.str();
  }

  void log_startup_parameters()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Params | center_offset=%.4f deadband=%.4f max_linear_speed=%.3f max_angular_speed=%.3f max_joint_speed=%.3f",
      this->get_parameter("center_offset").as_double(),
      this->get_parameter("deadband").as_double(),
      this->get_parameter("max_linear_speed").as_double(),
      this->get_parameter("max_angular_speed").as_double(),
      this->get_parameter("max_joint_speed").as_double());

    RCLCPP_INFO(
      this->get_logger(),
      "Params | pose_twitch_gain=%.3f pose_twitch_ticks=%ld",
      this->get_parameter("pose_twitch_gain").as_double(),
      this->get_parameter("pose_twitch_ticks").as_int());

    RCLCPP_INFO(
      this->get_logger(),
      "Params | ee_position_step=%.4f ee_deadband=%.3f",
      this->get_parameter("ee_position_step").as_double(),
      this->get_parameter("ee_deadband").as_double());

    RCLCPP_INFO(
      this->get_logger(),
      "Params | command_dt_fallback=%.4f command_dt_min=%.4f command_dt_max=%.4f",
      this->get_parameter("command_dt_fallback").as_double(),
      this->get_parameter("command_dt_min").as_double(),
      this->get_parameter("command_dt_max").as_double());

    RCLCPP_INFO(
      this->get_logger(),
      "Params | joint_state_timeout_sec=%.3f resume_neutral_threshold=%.3f",
      this->get_parameter("joint_state_timeout_sec").as_double(),
      this->get_parameter("resume_neutral_threshold").as_double());

    RCLCPP_INFO(
      this->get_logger(),
      "Params | publish_rate_hz=%.2f",
      this->get_parameter("publish_rate_hz").as_double());

    RCLCPP_INFO(
      this->get_logger(),
      "Params | blocked_diff_threshold=%.3f blocked_consecutive_cycles=%ld",
      this->get_parameter("blocked_diff_threshold").as_double(),
      this->get_parameter("blocked_consecutive_cycles").as_int());

    RCLCPP_INFO(
      this->get_logger(),
      "Topics | joint_state=%s | left_arm=%s | right_arm=%s | head=%s | lift=%s",
      this->get_parameter("joint_state_topic").as_string().c_str(),
      this->get_parameter("left_arm_topic").as_string().c_str(),
      this->get_parameter("right_arm_topic").as_string().c_str(),
      this->get_parameter("head_topic").as_string().c_str(),
      this->get_parameter("lift_topic").as_string().c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Limits left lower=%s upper=%s",
      vector_to_string(this->get_parameter("left_arm_lower_limits").as_double_array()).c_str(),
      vector_to_string(this->get_parameter("left_arm_upper_limits").as_double_array()).c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Limits right lower=%s upper=%s",
      vector_to_string(this->get_parameter("right_arm_lower_limits").as_double_array()).c_str(),
      vector_to_string(this->get_parameter("right_arm_upper_limits").as_double_array()).c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Limits head lower=%s upper=%s",
      vector_to_string(this->get_parameter("head_lower_limits").as_double_array()).c_str(),
      vector_to_string(this->get_parameter("head_upper_limits").as_double_array()).c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Limits lift lower=%s upper=%s",
      vector_to_string(this->get_parameter("lift_lower_limits").as_double_array()).c_str(),
      vector_to_string(this->get_parameter("lift_upper_limits").as_double_array()).c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "IK runtime | enabled=%s",
      ik_runtime_ready_ ? "true" : "false");
  }

  struct JointBinding
  {
    size_t cmd_index {0};
    int qpos_adr {-1};
  };

  void create_joint_bindings(
    const std::vector<std::string> & joint_names,
    std::vector<JointBinding> & bindings,
    const std::string & group_label)
  {
    bindings.clear();
    if (!ik_model_) {
      return;
    }

    for (size_t i = 0; i < joint_names.size(); ++i) {
      const auto & name = joint_names[i];
      const int joint_id = mj_name2id(ik_model_.get(), mjOBJ_JOINT, name.c_str());
      if (joint_id < 0) {
        const bool is_gripper_joint = (name.find("gripper_") == 0);
        if (!is_gripper_joint) {
          RCLCPP_WARN(
            this->get_logger(),
            "IK model missing joint '%s' for %s",
            name.c_str(),
            group_label.c_str());
        }
        continue;
      }

      const int qpos_adr = ik_model_->jnt_qposadr[joint_id];
      if (qpos_adr < 0 || qpos_adr >= ik_model_->nq) {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid qpos adr for joint '%s' (%d)",
          name.c_str(),
          qpos_adr);
        continue;
      }

      bindings.push_back(JointBinding {i, qpos_adr});
    }
  }

  void apply_group_to_mujoco(
    const std::vector<JointBinding> & bindings,
    const std::vector<double> & source_positions)
  {
    if (!ik_data_) {
      return;
    }

    for (const auto & b : bindings) {
      if (b.cmd_index < source_positions.size() && b.qpos_adr >= 0 && b.qpos_adr < ik_model_->nq) {
        ik_data_->qpos[b.qpos_adr] = source_positions[b.cmd_index];
      }
    }
  }

  void extract_group_from_mujoco(
    const std::vector<JointBinding> & bindings,
    std::vector<double> & target_positions)
  {
    if (!ik_data_) {
      return;
    }

    for (const auto & b : bindings) {
      if (b.cmd_index < target_positions.size() && b.qpos_adr >= 0 && b.qpos_adr < ik_model_->nq) {
        target_positions[b.cmd_index] = ik_data_->qpos[b.qpos_adr];
      }
    }
  }

  bool apply_kinematic_state_to_mujoco()
  {
    if (!ik_runtime_ready_ || !ik_model_ || !ik_data_) {
      return false;
    }

    mju_zero(ik_data_->qvel, ik_model_->nv);

    apply_group_to_mujoco(left_arm_joint_bindings_, left_arm_kin_pos_);
    apply_group_to_mujoco(right_arm_joint_bindings_, right_arm_kin_pos_);
    apply_group_to_mujoco(head_joint_bindings_, head_kin_pos_);
    apply_group_to_mujoco(lift_joint_bindings_, lift_kin_pos_);

    mj_forward(ik_model_.get(), ik_data_.get());
    return true;
  }

  void sync_command_state_from_mujoco()
  {
    if (!ik_runtime_ready_ || !ik_data_) {
      return;
    }

    extract_group_from_mujoco(left_arm_joint_bindings_, left_arm_cmd_pos_);
    extract_group_from_mujoco(right_arm_joint_bindings_, right_arm_cmd_pos_);
    extract_group_from_mujoco(head_joint_bindings_, head_cmd_pos_);
    extract_group_from_mujoco(lift_joint_bindings_, lift_cmd_pos_);
  }

  bool initialize_ee_goals_from_kinematic_state()
  {
    if (!apply_kinematic_state_to_mujoco()) {
      return false;
    }

    if (ik_left_site_id_ < 0 || ik_right_site_id_ < 0) {
      return false;
    }

    left_ee_goal_ = Eigen::Vector3d::Map(ik_data_->site_xpos + 3 * ik_left_site_id_);
    right_ee_goal_ = Eigen::Vector3d::Map(ik_data_->site_xpos + 3 * ik_right_site_id_);
    ee_goal_initialized_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Initialized EE goals from MuJoCo sites | L:[%.3f %.3f %.3f] R:[%.3f %.3f %.3f]",
      left_ee_goal_[0],
      left_ee_goal_[1],
      left_ee_goal_[2],
      right_ee_goal_[0],
      right_ee_goal_[1],
      right_ee_goal_[2]);

    return true;
  }

  void initialize_ik_runtime()
  {
    using ament_index_cpp::get_package_share_directory;

    std::string xml_path;
    try {
      xml_path =
        get_package_share_directory("ffw_collision_checker") +
        "/3rd_party/robotis_ffw/scene_inverse_kinematic.xml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "IK init failed (package lookup): %s", e.what());
      return;
    }

    char error[1000] = {0};
    mjModel * raw_model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (raw_model == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "IK init failed (mj_loadXML): %s", error);
      return;
    }

    ik_model_.reset(raw_model);
    ik_data_.reset(mj_makeData(ik_model_.get()));
    if (!ik_data_) {
      RCLCPP_ERROR(this->get_logger(), "IK init failed (mj_makeData)");
      ik_model_.reset();
      return;
    }

    mju_zero(ik_data_->qpos, ik_model_->nq);
    mju_zero(ik_data_->qvel, ik_model_->nv);
    if (ik_model_->nq >= 7 && ik_model_->jnt_type[0] == mjJNT_FREE) {
      ik_data_->qpos[3] = 1.0;
    }
    mj_forward(ik_model_.get(), ik_data_.get());

    ik_solver_ = std::make_unique<ffw_ik::IKSolver>(ik_model_.get());

    ik_solver_cfg_.damping = 1e-3;
    ik_solver_cfg_.step_size = 0.15;
    ik_solver_cfg_.tolerance = 5e-3;
    ik_solver_cfg_.joint_vel_limit = 3.1;
    ik_solver_cfg_.max_steps = 100;
    ik_solver_cfg_.topk_contacts = 5;
    ik_solver_cfg_.ee_window = 5;
    ik_solver_cfg_.ee_improvement_rate = 0.02;
    ik_solver_cfg_.dist_window = 5;
    ik_solver_cfg_.dist_stability_thresh = 0.002;
    ik_solver_cfg_.dist_safe_ratio = 0.98;
    ik_solver_cfg_.early_convergence_obj = 0.87;

    ik_collision_cfg_.collision_margin = 0.10;
    ik_collision_cfg_.weight_scale = 0.005;
    ik_collision_cfg_.epsilon = 1e-1;

    create_joint_bindings(left_arm_joint_names_, left_arm_joint_bindings_, "left_arm");
    create_joint_bindings(right_arm_joint_names_, right_arm_joint_bindings_, "right_arm");
    create_joint_bindings(head_joint_names_, head_joint_bindings_, "head");
    create_joint_bindings(lift_joint_names_, lift_joint_bindings_, "lift");

    ik_left_site_id_ = mj_name2id(ik_model_.get(), mjOBJ_SITE, "left_gripper_site");
    ik_right_site_id_ = mj_name2id(ik_model_.get(), mjOBJ_SITE, "right_gripper_site");

    if (ik_left_site_id_ < 0 || ik_right_site_id_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "IK init failed: gripper sites not found");
      ik_solver_.reset();
      ik_data_.reset();
      ik_model_.reset();
      return;
    }

    ik_runtime_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "IK runtime initialized from: %s", xml_path.c_str());
  }

  static const char * objective_to_string(int objective_raw)
  {
    switch (objective_raw) {
      case -1:
        return "END_EFFECTOR";
      case 0:
        return "POSE";
      case 1:
        return "NAVIGATE";
      default:
        return "UNKNOWN";
    }
  }

  static const char * kill_to_string(int kill_raw)
  {
    return (kill_raw == static_cast<int>(KillSwitch::KILL)) ? "KILL" : "RUN";
  }

  static const char * trigger_to_string(int trigger_raw)
  {
    return (trigger_raw == static_cast<int>(Trigger::ON)) ? "ON" : "OFF";
  }

  static const char * selector_to_string(int selector_raw)
  {
    return (selector_raw == static_cast<int>(Selector::PRIMARY)) ? "PRIMARY" : "SECONDARY";
  }

  static const char * pose_level_to_string(int level)
  {
    switch (level) {
      case 0:
        return "JOINT1";
      case 1:
        return "JOINT2";
      case 2:
        return "JOINT3";
      case 3:
        return "JOINT4";
      case 4:
        return "JOINT5";
      case 5:
        return "JOINT6";
      case 6:
        return "JOINT7";
      case 7:
        return "GRIPPER";
      default:
        return "UNKNOWN";
    }
  }

  void notify_state_changes(
    int objective_raw,
    int kill_raw,
    int trigger_raw,
    int selector_raw)
  {
    if (!state_initialized_) {
      prev_objective_raw_ = objective_raw;
      prev_kill_raw_ = kill_raw;
      prev_trigger_raw_ = trigger_raw;
      prev_selector_raw_ = selector_raw;
      state_initialized_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Initial state | MODE=%s | KILL=%s | TRIGGER=%s | SELECTOR=%s",
        objective_to_string(objective_raw),
        kill_to_string(kill_raw),
        trigger_to_string(trigger_raw),
        selector_to_string(selector_raw));
      return;
    }

    if (objective_raw != prev_objective_raw_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Mode switched: %s -> %s",
        objective_to_string(prev_objective_raw_),
        objective_to_string(objective_raw));

      // Any mode transition invalidates previous EE goal context.
      ee_goal_initialized_ = false;

      if (objective_raw == static_cast<int>(Objective::END_EFFECTOR)) {
        // Re-seed EE goals from the current measured pose whenever entering EE mode.
        if (ik_runtime_ready_ && joint_state_initialized_) {
          (void)initialize_ee_goals_from_kinematic_state();
        }
      }

      prev_objective_raw_ = objective_raw;
    }

    if (kill_raw != prev_kill_raw_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Kill switch changed: %s -> %s",
        kill_to_string(prev_kill_raw_),
        kill_to_string(kill_raw));
      prev_kill_raw_ = kill_raw;
    }

    if (trigger_raw != prev_trigger_raw_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Trigger changed: %s -> %s",
        trigger_to_string(prev_trigger_raw_),
        trigger_to_string(trigger_raw));
      prev_trigger_raw_ = trigger_raw;
    }

    if (selector_raw != prev_selector_raw_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Drive selector changed: %s -> %s",
        selector_to_string(prev_selector_raw_),
        selector_to_string(selector_raw));
      prev_selector_raw_ = selector_raw;
    }
  }

  struct DecodedInput
  {
    float lx {0.0f};
    float ly {0.0f};
    float rx {0.0f};
    float ry {0.0f};
    float lz {0.0f};
    float rz {0.0f};
    std::array<int, 5> b {{-1, -1, -1, -1, -1}};
    std::array<int, 3> t {{0, 0, 0}};
  };

  static bool update_group_kinematics_from_joint_state(
    const sensor_msgs::msg::JointState & msg,
    const std::unordered_map<std::string, size_t> & joint_index_map,
    const std::vector<std::string> & joint_names,
    std::vector<double> & kinematic_positions)
  {
    if (kinematic_positions.size() != joint_names.size()) {
      kinematic_positions.assign(joint_names.size(), 0.0);
    }

    bool any_updated = false;
    const size_t position_count = msg.position.size();

    for (size_t i = 0; i < joint_names.size(); ++i) {
      const auto & target_joint = joint_names[i];
      const auto it = joint_index_map.find(target_joint);
      if (it != joint_index_map.end() && it->second < position_count) {
        kinematic_positions[i] = msg.position[it->second];
        any_updated = true;
      }
    }

    return any_updated;
  }

  static double max_abs_diff(
    const std::vector<double> & a,
    const std::vector<double> & b)
  {
    const size_t n = std::min(a.size(), b.size());
    double max_d = 0.0;
    for (size_t i = 0; i < n; ++i) {
      max_d = std::max(max_d, std::fabs(a[i] - b[i]));
    }
    return max_d;
  }

  void resync_command_state_from_kinematic_state()
  {
    left_arm_cmd_pos_ = left_arm_kin_pos_;
    right_arm_cmd_pos_ = right_arm_kin_pos_;
    head_cmd_pos_ = head_kin_pos_;
    lift_cmd_pos_ = lift_kin_pos_;

    if (ik_runtime_ready_) {
      (void)apply_kinematic_state_to_mujoco();
      ee_goal_initialized_ = false;
      (void)initialize_ee_goals_from_kinematic_state();
    }
  }

  bool is_input_neutral(const DecodedInput & in) const
  {
    const double threshold = this->get_parameter("resume_neutral_threshold").as_double();
    return
      std::fabs(in.lx) <= threshold &&
      std::fabs(in.ly) <= threshold &&
      std::fabs(in.rx) <= threshold &&
      std::fabs(in.ry) <= threshold &&
      std::fabs(in.lz) <= threshold &&
      std::fabs(in.rz) <= threshold;
  }

  bool update_joint_state_health_and_gate()
  {
    if (!joint_state_initialized_ || !joint_state_stamp_initialized_) {
      command_publish_enabled_ = false;
      return false;
    }

    const double timeout_sec = std::max(0.1, this->get_parameter("joint_state_timeout_sec").as_double());
    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> age = now - last_joint_state_stamp_;
    const bool stale_now = age.count() > timeout_sec;

    if (stale_now) {
      if (!joint_state_stale_) {
        joint_state_stale_ = true;
        command_publish_enabled_ = false;
        resync_command_state_from_kinematic_state();
        RCLCPP_WARN(
          this->get_logger(),
          "Joint state timeout (%.3fs > %.3fs). Command publishing paused.",
          age.count(),
          timeout_sec);
      }
      return false;
    }

    return true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty() || msg->position.empty()) {
      return;
    }

    std::unordered_map<std::string, size_t> joint_index_map;
    joint_index_map.reserve(msg->name.size());
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_index_map.emplace(msg->name[i], i);
    }

    bool any_group_updated = false;
    any_group_updated |=
      update_group_kinematics_from_joint_state(*msg, joint_index_map, left_arm_joint_names_, left_arm_kin_pos_);
    any_group_updated |=
      update_group_kinematics_from_joint_state(*msg, joint_index_map, right_arm_joint_names_, right_arm_kin_pos_);
    any_group_updated |=
      update_group_kinematics_from_joint_state(*msg, joint_index_map, head_joint_names_, head_kin_pos_);
    any_group_updated |=
      update_group_kinematics_from_joint_state(*msg, joint_index_map, lift_joint_names_, lift_kin_pos_);

    if (!any_group_updated) {
      return;
    }

    last_joint_state_stamp_ = std::chrono::steady_clock::now();
    joint_state_stamp_initialized_ = true;

    if (joint_state_stale_) {
      joint_state_stale_ = false;
      command_publish_enabled_ = false;
      resync_command_state_from_kinematic_state();
      RCLCPP_INFO(
        this->get_logger(),
        "Joint state stream restored. Resynced internal command state; waiting for KILL->RUN with neutral sticks.");
    }

    if (!joint_state_initialized_) {
      resync_command_state_from_kinematic_state();
      joint_state_initialized_ = true;

      RCLCPP_INFO(this->get_logger(), "Initialized command pose from kinematic joint feedback.");

      if (ik_runtime_ready_ && !ee_goal_initialized_) {
        (void)initialize_ee_goals_from_kinematic_state();
      }
      return;
    }

    if (ik_runtime_ready_) {
      (void)apply_kinematic_state_to_mujoco();
    }

    left_arm_max_diff_ = max_abs_diff(left_arm_cmd_pos_, left_arm_kin_pos_);
    right_arm_max_diff_ = max_abs_diff(right_arm_cmd_pos_, right_arm_kin_pos_);
    head_max_diff_ = max_abs_diff(head_cmd_pos_, head_kin_pos_);
    lift_max_diff_ = max_abs_diff(lift_cmd_pos_, lift_kin_pos_);

    const double overall_max_diff = std::max(
      std::max(left_arm_max_diff_, right_arm_max_diff_),
      std::max(head_max_diff_, lift_max_diff_));

    const double diff_threshold = this->get_parameter("blocked_diff_threshold").as_double();
    const int required_cycles = std::max(
      1,
      static_cast<int>(this->get_parameter("blocked_consecutive_cycles").as_int()));

    if (overall_max_diff > diff_threshold) {
      ++blocked_cycle_count_;
    } else {
      blocked_cycle_count_ = 0;
    }

    if (blocked_cycle_count_ >= required_cycles) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Possible blocked motion (max |cmd-kin|=%.4f rad/m) | L=%.4f R=%.4f H=%.4f Lift=%.4f",
        overall_max_diff,
        left_arm_max_diff_,
        right_arm_max_diff_,
        head_max_diff_,
        lift_max_diff_);
    }
  }

  void update_command_dt_from_callback_period()
  {
    const auto now = std::chrono::steady_clock::now();
    if (!command_dt_last_stamp_initialized_) {
      command_dt_last_stamp_ = now;
      command_dt_last_stamp_initialized_ = true;
      return;
    }

    const std::chrono::duration<double> dt = now - command_dt_last_stamp_;
    command_dt_last_stamp_ = now;

    if (dt.count() <= 1e-6) {
      return;
    }

    const double dt_min = this->get_parameter("command_dt_min").as_double();
    const double dt_max = this->get_parameter("command_dt_max").as_double();
    command_dt_ = std::clamp(dt.count(), dt_min, dt_max);
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(latest_joy_mutex_);
    latest_joy_msg_ = *msg;
    latest_joy_received_ = true;
  }

  void control_timer_callback()
  {
    sensor_msgs::msg::Joy joy_msg;
    {
      std::lock_guard<std::mutex> lock(latest_joy_mutex_);
      if (!latest_joy_received_) {
        return;
      }
      joy_msg = latest_joy_msg_;
    }

    update_command_dt_from_callback_period();

    if (joy_msg.axes.size() < 8) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Expected at least 8 joystick axes, got %zu", joy_msg.axes.size());
      return;
    }

    const auto input = decode_input(joy_msg);

    geometry_msgs::msg::Twist base_cmd;
    geometry_msgs::msg::Vector3Stamped linear_msg;
    std::vector<double> left_arm_vel(left_arm_joint_names_.size(), 0.0);
    std::vector<double> right_arm_vel(right_arm_joint_names_.size(), 0.0);
    std::vector<double> head_vel(head_joint_names_.size(), 0.0);
    std::vector<double> lift_vel(lift_joint_names_.size(), 0.0);

    linear_msg.header.stamp = this->now();
    linear_msg.header.frame_id = "base_link";

    const int objective_raw = input.t[1];  // T1 = mode selector from dashboard convention
    const int kill_raw = input.b[1];       // B1

    const auto objective = static_cast<Objective>(objective_raw);
    const bool kill_active = (kill_raw == static_cast<int>(KillSwitch::KILL));

    bool kill_to_run_edge = false;
    if (kill_gate_initialized_) {
      kill_to_run_edge =
        (last_kill_gate_raw_ == static_cast<int>(KillSwitch::KILL)) &&
        (kill_raw == static_cast<int>(KillSwitch::RUN));
    }
    last_kill_gate_raw_ = kill_raw;
    kill_gate_initialized_ = true;

    notify_state_changes(objective_raw, kill_raw, input.b[0], input.b[3]);

    if (kill_active) {
      command_publish_enabled_ = false;
      return;
    }

    if (!update_joint_state_health_and_gate()) {
      return;
    }

    if (!command_publish_enabled_) {
      if (kill_to_run_edge && is_input_neutral(input)) {
        command_publish_enabled_ = true;
        resync_command_state_from_kinematic_state();
        RCLCPP_INFO(
          this->get_logger(),
          "Command publishing resumed after KILL->RUN edge with neutral joystick.");
      } else {
        return;
      }
    }

    if (!joint_state_initialized_) {
      return;
    }

    update_gripper_toggles(input.b[4], input.b[2]);
    handle_b0_edge_trigger(input.b[0], objective);

    switch (objective) {
      case Objective::NAVIGATE:
        process_navigate_mode(input, base_cmd);
        break;
      case Objective::POSE:
        process_pose_mode(input, left_arm_vel, right_arm_vel, head_vel, lift_vel);
        break;
      case Objective::END_EFFECTOR:
      default:
        process_end_effector_mode(input);
        break;
    }

    linear_msg.vector.x = base_cmd.linear.x;
    linear_msg.vector.y = base_cmd.linear.y;
    linear_msg.vector.z = base_cmd.linear.z;

    publish_outputs(base_cmd, linear_msg, left_arm_vel, right_arm_vel, head_vel, lift_vel);
  }

  DecodedInput decode_input(const sensor_msgs::msg::Joy & joy)
  {
    DecodedInput out;

    const float offset = static_cast<float>(this->get_parameter("center_offset").as_double());
    const float deadband = static_cast<float>(this->get_parameter("deadband").as_double());

    auto clean_axis = [offset](float v) {
      return std::clamp(v - offset, -1.0f, 1.0f);
    };

    out.ry = apply_deadband(clean_axis(joy.axes[0]), deadband);
    out.lx = apply_deadband(clean_axis(joy.axes[1]), deadband);
    out.rx = apply_deadband(clean_axis(joy.axes[2]), deadband);
    out.ly = apply_deadband(clean_axis(joy.axes[3]), deadband);
    out.lz = apply_deadband(clean_axis(joy.axes[4]), deadband);
    out.rz = apply_deadband(clean_axis(joy.axes[5]), deadband);

    constexpr float kUnitLength = 2.76f;
    const auto b_states = decode_universal_switches(clean_axis(joy.axes[6]), kUnitLength, 2, 5);
    const auto t_states = decode_universal_switches(clean_axis(joy.axes[7]), kUnitLength, 3, 3);

    for (size_t i = 0; i < out.b.size() && i < b_states.size(); ++i) {
      out.b[i] = b_states[i];
    }

    for (size_t i = 0; i < out.t.size() && i < t_states.size(); ++i) {
      out.t[i] = t_states[i];
    }

    return out;
  }

  void process_navigate_mode(const DecodedInput & in, geometry_msgs::msg::Twist & base_cmd)
  {
    const double max_linear = this->get_parameter("max_linear_speed").as_double();
    const double max_angular = this->get_parameter("max_angular_speed").as_double();

    const bool primary_selector = (in.b[3] == static_cast<int>(Selector::PRIMARY));

    if (primary_selector) {
      // Holonomic: VX <- LX, VY <- LY, Yaw <- RY
      base_cmd.linear.x = max_linear * in.lx;
      base_cmd.linear.y = max_linear * in.ly;
      base_cmd.angular.z = max_angular * in.ry;
    } else {
      // Secondary: tank-like fallback mapped to 3-wheel swerve (LX/RX differential)
      const float left = in.lx;
      const float right = in.rx;

      base_cmd.linear.x = max_linear * ((left + right) * 0.5f);
      base_cmd.linear.y = 0.0;
      base_cmd.angular.z = max_angular * ((right - left) * 0.5f);
    }
  }

  void process_pose_mode(
    const DecodedInput & in,
    std::vector<double> & left_arm_vel,
    std::vector<double> & right_arm_vel,
    std::vector<double> & head_vel,
    std::vector<double> & lift_vel)
  {
    const double max_joint = this->get_parameter("max_joint_speed").as_double();

    // Always-available pose channels
    lift_vel[0] = max_joint * in.lx;
    head_vel[0] = max_joint * in.rx;
    head_vel[1] = max_joint * in.ry;

    const double l_adjust = -max_joint * in.lz;
    const double r_adjust = -max_joint * in.rz;

    // Active joint level cycling across 8 levels: 7 joints + gripper
    if (active_joint_level_ >= 0 && active_joint_level_ < 8) {
      left_arm_vel[active_joint_level_] = l_adjust;
      right_arm_vel[active_joint_level_] = r_adjust;
    }

    apply_pose_level_twitch(left_arm_vel, right_arm_vel, std::fabs(l_adjust) > 1e-4 || std::fabs(r_adjust) > 1e-4);
  }

  void process_end_effector_mode(const DecodedInput & in)
  {
    if (!ik_runtime_ready_ || !ik_solver_ || !ik_data_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "EE mode requested but IK runtime is not ready");
      return;
    }

    if (!joint_state_initialized_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "EE mode waiting for joint state initialization");
      return;
    }

    if (!ee_goal_initialized_ && !initialize_ee_goals_from_kinematic_state()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "EE mode failed to initialize goal from kinematic state");
      return;
    }

    const double step = this->get_parameter("ee_position_step").as_double();
    const double ee_deadband = this->get_parameter("ee_deadband").as_double();

    const double lx = apply_deadband(in.lx, static_cast<float>(ee_deadband));
    const double ly = apply_deadband(in.ly, static_cast<float>(ee_deadband));
    const double lz = apply_deadband(in.lz, static_cast<float>(ee_deadband));
    const double rx = apply_deadband(in.rx, static_cast<float>(ee_deadband));
    const double ry = apply_deadband(in.ry, static_cast<float>(ee_deadband));
    const double rz = apply_deadband(in.rz, static_cast<float>(ee_deadband));

    left_ee_goal_[0] += step * lx;
    left_ee_goal_[1] += step * ly;
    left_ee_goal_[2] += step * lz;

    right_ee_goal_[0] += step * rx;
    right_ee_goal_[1] += step * ry;
    right_ee_goal_[2] += step * rz;

    if (!apply_kinematic_state_to_mujoco()) {
      return;
    }

    const auto trajectory = ik_solver_->solve(
      ik_data_.get(),
      left_ee_goal_,
      right_ee_goal_,
      ik_solver_cfg_,
      ik_collision_cfg_);

    if (trajectory.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "IK returned empty trajectory in EE mode");
      return;
    }

    sync_command_state_from_mujoco();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "EE goal | L:[%.3f %.3f %.3f] R:[%.3f %.3f %.3f] | IK steps=%zu",
      left_ee_goal_[0],
      left_ee_goal_[1],
      left_ee_goal_[2],
      right_ee_goal_[0],
      right_ee_goal_[1],
      right_ee_goal_[2],
      trajectory.size());
  }

  void update_gripper_toggles(int b4_raw, int b2_raw)
  {
    if (!gripper_state_initialized_) {
      prev_b4_raw_ = b4_raw;
      prev_b2_raw_ = b2_raw;
      gripper_state_initialized_ = true;
      return;
    }

    if (b4_raw != prev_b4_raw_) {
      left_gripper_closed_ = !left_gripper_closed_;
      RCLCPP_INFO(
        this->get_logger(),
        "Left gripper toggled: %s",
        left_gripper_closed_ ? "CLOSED" : "OPEN");
      prev_b4_raw_ = b4_raw;
    }

    if (b2_raw != prev_b2_raw_) {
      right_gripper_closed_ = !right_gripper_closed_;
      RCLCPP_INFO(
        this->get_logger(),
        "Right gripper toggled: %s",
        right_gripper_closed_ ? "CLOSED" : "OPEN");
      prev_b2_raw_ = b2_raw;
    }
  }

  void handle_b0_edge_trigger(int b0_raw, Objective objective)
  {
    const bool trigger_on = (b0_raw == static_cast<int>(Trigger::ON));
    if (objective == Objective::POSE && trigger_on && !prev_trigger_on_) {
      active_joint_level_ = (active_joint_level_ + 1) % 8;
      twitch_level_index_ = active_joint_level_;
      const int configured_ticks = static_cast<int>(this->get_parameter("pose_twitch_ticks").as_int());
      twitch_ticks_remaining_ = std::max(2, configured_ticks);
      twitch_half_ticks_ = std::max(1, twitch_ticks_remaining_ / 2);
      RCLCPP_INFO(
        this->get_logger(),
        "Pose joint level changed to %d (%s)",
        active_joint_level_,
        pose_level_to_string(active_joint_level_));
    }
    prev_trigger_on_ = trigger_on;
  }

  void apply_pose_level_twitch(
    std::vector<double> & left_arm_vel,
    std::vector<double> & right_arm_vel,
    bool user_is_actively_adjusting)
  {
    if (twitch_ticks_remaining_ <= 0 || twitch_level_index_ < 0 || twitch_level_index_ >= 8) {
      return;
    }

    // Avoid fighting operator input while actively moving LZ/RZ.
    if (user_is_actively_adjusting) {
      return;
    }

    const double max_joint = this->get_parameter("max_joint_speed").as_double();
    const double twitch_gain = this->get_parameter("pose_twitch_gain").as_double();
    const double twitch_amp = twitch_gain * max_joint;

    // First half positive, second half negative -> visible nudge that settles back.
    const double sign = (twitch_ticks_remaining_ > twitch_half_ticks_) ? 1.0 : -1.0;
    left_arm_vel[twitch_level_index_] += sign * twitch_amp;
    right_arm_vel[twitch_level_index_] += sign * twitch_amp;

    --twitch_ticks_remaining_;
  }

  void publish_joint_command(
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & publisher,
    const std::vector<std::string> & joint_names,
    std::vector<double> & command_positions,
    const std::vector<double> & lower_limits,
    const std::vector<double> & upper_limits,
    const std::vector<double> & velocities)
  {
    if (command_positions.size() != velocities.size()) {
      command_positions.assign(velocities.size(), 0.0);
    }

    for (size_t i = 0; i < velocities.size(); ++i) {
      command_positions[i] += velocities[i] * command_dt_;
      command_positions[i] = std::clamp(command_positions[i], lower_limits[i], upper_limits[i]);
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = command_positions;
    point.velocities = velocities;
    const double tf = std::max(1e-4, command_dt_);
    const int32_t sec = static_cast<int32_t>(tf);
    const uint32_t nsec = static_cast<uint32_t>((tf - static_cast<double>(sec)) * 1e9);
    point.time_from_start.sec = sec;
    point.time_from_start.nanosec = nsec;

    traj.points.push_back(point);
    publisher->publish(traj);
  }

  void publish_outputs(
    const geometry_msgs::msg::Twist & base_cmd,
    const geometry_msgs::msg::Vector3Stamped & linear_msg,
    const std::vector<double> & left_arm_vel,
    const std::vector<double> & right_arm_vel,
    const std::vector<double> & head_vel,
    const std::vector<double> & lift_vel)
  {
    base_vel_pub_->publish(base_cmd);
    linear_vel_pub_->publish(linear_msg);

    if (!joint_state_initialized_) {
      return;
    }

    publish_joint_command(
      left_arm_pub_,
      left_arm_joint_names_,
      left_arm_cmd_pos_,
      left_arm_lower_limits_,
      left_arm_upper_limits_,
      left_arm_vel);
    publish_joint_command(
      right_arm_pub_,
      right_arm_joint_names_,
      right_arm_cmd_pos_,
      right_arm_lower_limits_,
      right_arm_upper_limits_,
      right_arm_vel);
    publish_joint_command(
      head_pub_,
      head_joint_names_,
      head_cmd_pos_,
      head_lower_limits_,
      head_upper_limits_,
      head_vel);
    publish_joint_command(
      lift_pub_,
      lift_joint_names_,
      lift_cmd_pos_,
      lift_lower_limits_,
      lift_upper_limits_,
      lift_vel);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr linear_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lift_pub_;

  std::vector<std::string> left_arm_joint_names_;
  std::vector<std::string> right_arm_joint_names_;
  std::vector<std::string> head_joint_names_;
  std::vector<std::string> lift_joint_names_;

  std::vector<double> left_arm_cmd_pos_;
  std::vector<double> right_arm_cmd_pos_;
  std::vector<double> head_cmd_pos_;
  std::vector<double> lift_cmd_pos_;

  std::vector<double> left_arm_kin_pos_;
  std::vector<double> right_arm_kin_pos_;
  std::vector<double> head_kin_pos_;
  std::vector<double> lift_kin_pos_;

  std::vector<double> left_arm_lower_limits_;
  std::vector<double> left_arm_upper_limits_;
  std::vector<double> right_arm_lower_limits_;
  std::vector<double> right_arm_upper_limits_;
  std::vector<double> head_lower_limits_;
  std::vector<double> head_upper_limits_;
  std::vector<double> lift_lower_limits_;
  std::vector<double> lift_upper_limits_;

  std::unique_ptr<mjModel, MjModelDeleter> ik_model_;
  std::unique_ptr<mjData, MjDataDeleter> ik_data_;
  std::unique_ptr<ffw_ik::IKSolver> ik_solver_;
  ffw_ik::SolverConfig ik_solver_cfg_;
  ffw_ik::CollisionCostConfig ik_collision_cfg_;
  std::vector<JointBinding> left_arm_joint_bindings_;
  std::vector<JointBinding> right_arm_joint_bindings_;
  std::vector<JointBinding> head_joint_bindings_;
  std::vector<JointBinding> lift_joint_bindings_;
  int ik_left_site_id_ {-1};
  int ik_right_site_id_ {-1};
  bool ik_runtime_ready_ {false};
  bool ee_goal_initialized_ {false};
  Eigen::Vector3d left_ee_goal_ {0.0, 0.0, 0.0};
  Eigen::Vector3d right_ee_goal_ {0.0, 0.0, 0.0};

  double command_dt_ {0.02};
  bool command_dt_last_stamp_initialized_ {false};
  std::chrono::steady_clock::time_point command_dt_last_stamp_ {};
  std::mutex latest_joy_mutex_;
  sensor_msgs::msg::Joy latest_joy_msg_;
  bool latest_joy_received_ {false};

  bool joint_state_initialized_ {false};
  int blocked_cycle_count_ {0};
  double left_arm_max_diff_ {0.0};
  double right_arm_max_diff_ {0.0};
  double head_max_diff_ {0.0};
  double lift_max_diff_ {0.0};

  bool state_initialized_ {false};
  int prev_objective_raw_ {0};
  int prev_kill_raw_ {1};
  int prev_trigger_raw_ {1};
  int prev_selector_raw_ {1};
  bool gripper_state_initialized_ {false};
  int prev_b4_raw_ {1};
  int prev_b2_raw_ {1};
  bool left_gripper_closed_ {false};
  bool right_gripper_closed_ {false};
  int twitch_level_index_ {-1};
  int twitch_half_ticks_ {3};
  int twitch_ticks_remaining_ {0};
  bool prev_trigger_on_ {false};
  int active_joint_level_ {0};
  bool joint_state_stamp_initialized_ {false};
  std::chrono::steady_clock::time_point last_joint_state_stamp_ {};
  bool joint_state_stale_ {false};
  bool command_publish_enabled_ {false};
  bool kill_gate_initialized_ {false};
  int last_kill_gate_raw_ {static_cast<int>(KillSwitch::KILL)};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadioControlMapper>());
  rclcpp::shutdown();
  return 0;
}
