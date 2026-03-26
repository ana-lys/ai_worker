#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
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

    command_dt_ = 1.0 / 50.0;  // Match joy autorepeat_rate default in launch file

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
      "Topics | left_arm=%s | right_arm=%s | head=%s | lift=%s",
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

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() < 8) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Expected at least 8 joystick axes, got %zu", msg->axes.size());
      return;
    }

    const auto input = decode_input(*msg);

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

    notify_state_changes(objective_raw, kill_raw, input.b[0], input.b[3]);

    if (kill_active) {
      publish_outputs(base_cmd, linear_msg, left_arm_vel, right_arm_vel, head_vel, lift_vel);
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
        return;
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
    constexpr double step = 0.01;

    // Dummy EE pose integrator (arbitrary for testing)
    left_ee_pose_[0] += step * in.lx;
    left_ee_pose_[1] += step * in.ly;
    left_ee_pose_[2] += step * in.lz;

    right_ee_pose_[0] += step * in.rx;
    right_ee_pose_[1] += step * in.ry;
    right_ee_pose_[2] += step * in.rz;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "EE dummy pose | L:[%.3f %.3f %.3f] R:[%.3f %.3f %.3f]",
      left_ee_pose_[0], left_ee_pose_[1], left_ee_pose_[2],
      right_ee_pose_[0], right_ee_pose_[1], right_ee_pose_[2]);
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
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 20000000;  // 20 ms

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

  std::vector<double> left_arm_lower_limits_;
  std::vector<double> left_arm_upper_limits_;
  std::vector<double> right_arm_lower_limits_;
  std::vector<double> right_arm_upper_limits_;
  std::vector<double> head_lower_limits_;
  std::vector<double> head_upper_limits_;
  std::vector<double> lift_lower_limits_;
  std::vector<double> lift_upper_limits_;

  double command_dt_ {0.02};

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
  std::array<double, 3> left_ee_pose_ {{0.0, 0.0, 0.0}};
  std::array<double, 3> right_ee_pose_ {{0.0, 0.0, 0.0}};
  int twitch_level_index_ {-1};
  int twitch_half_ticks_ {3};
  int twitch_ticks_remaining_ {0};
  bool prev_trigger_on_ {false};
  int active_joint_level_ {0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadioControlMapper>());
  rclcpp::shutdown();
  return 0;
}
