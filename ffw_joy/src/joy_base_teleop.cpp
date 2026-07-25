#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;

// Simple double-click state tracker for SpaceMouse buttons
struct DoubleClickState {
  double last_press_time_sec = 0.0;
  int count = 0;
  bool prev_pressed = false;

  // Call on each button event; returns true when a double-click is detected
  bool update(bool pressed, double now, double timeout = 0.5)
  {
    if (pressed && !prev_pressed) {
      if ((now - last_press_time_sec) > timeout) {
        count = 1;
      } else {
        count++;
      }
      last_press_time_sec = now;
    }
    prev_pressed = pressed;
    return count >= 2;
  }

  void reset() { count = 0; }
};

// ─────────────────────────────────────────────────────────────────────────────
// JoyBaseTeleop
//
// Teleoperation for the robot base, elevator, and head using joystick input.
//
// All three devices work simultaneously — no device_type flag needed:
//
//   Logitech Extreme 3D Pro  (/joy)         → base cmd_vel + head/elevator
//     Stick X/Y  → cmd_vel linear x/y
//     Twist      → cmd_vel angular z
//     Throttle   → speed multiplier
//     Hat Y/X    → elevator Z / head pan
//     TRIGGER    → precision toggle
//     THUMB      → mode switch (BASE ↔ ARM ↔ LOGITECH)
//     BUTTON 2   → home base (zero velocity)
//
//   Right SpaceMouse  (/right/joy)           → right arm EE via joy_mapper
//     Button 0 double-click → right arm precision toggle
//     Button 1 double-click → mode switch
//
//   Left SpaceMouse   (/left/joy)            → left arm EE via joy_mapper
//     Button 0 double-click → left arm precision toggle
//     Button 1 double-click → mode switch
// ─────────────────────────────────────────────────────────────────────────────
class JoyBaseTeleop : public rclcpp::Node {
public:
  JoyBaseTeleop() : Node("joy_base_teleop") {
    // ── Common parameters ──────────────────────────────────────────────────
    this->declare_parameter("hardware_mode", true);
    hardware_mode_ = this->get_parameter("hardware_mode").as_bool();

    this->declare_parameter("sim", false);
    sim_ = this->get_parameter("sim").as_bool();

    // ── Always Logitech for base/head/elevator control ────────────────────
    declare_logitech_params();

    // Populate head limit vectors from declared parameters
    head_lower_limits_ = this->get_parameter("head_lower_limits").as_double_array();
    head_upper_limits_ = this->get_parameter("head_upper_limits").as_double_array();

    // ── Subscribers ─────────────────────────────────────────────────────────
    // Logitech → base cmd_vel + buttons
    base_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      this->get_parameter("base_joy_topic").as_string(), 10,
      std::bind(&JoyBaseTeleop::base_joy_callback, this, _1));

    // Logitech → head/elevator from hat switch (same topic, different callback)
    aux_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      this->get_parameter("aux_joy_topic").as_string(), 10,
      std::bind(&JoyBaseTeleop::aux_joy_callback, this, _1));

    // Logitech → store for EE mode (same topic, third subscriber is fine)
    logitech_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&JoyBaseTeleop::logitech_callback, this, _1));

    // Right SpaceMouse → button double-click for right arm precision
    right_sm_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/right/joy", 10,
      std::bind(&JoyBaseTeleop::right_sm_button_callback, this, _1));

    // Left SpaceMouse → button double-click for left arm precision
    left_sm_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/left/joy", 10,
      std::bind(&JoyBaseTeleop::left_sm_button_callback, this, _1));

    if (!sim_) {
      joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        this->get_parameter("joint_state_topic").as_string(), 20,
        std::bind(&JoyBaseTeleop::joint_state_callback, this, _1));
    }

    // ── Publishers ──────────────────────────────────────────────────────────
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    head_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("head_topic").as_string(), 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/teleop_mode", 10);

    right_arm_precision_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/spacemouse/right/precision_mode", 10);
    left_arm_precision_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/spacemouse/left/precision_mode", 10);

    // ── EE twist publisher (Logitech EE control in LOGITECH mode) ──────────
    ee_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/joy/ee_twist", 10);

    // Timer: publish EE commands from Logitech when in LOGITECH mode (20 Hz)
    logitech_ee_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&JoyBaseTeleop::publish_logitech_ee, this));

    // ── Initial state ───────────────────────────────────────────────────────
    current_head_pos_ = {0.0, 0.0};
    current_mode_ = "BASE";

    // Failsafe timer (checks every 2.5 seconds)
    failsafe_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2500),
      std::bind(&JoyBaseTeleop::failsafe_check, this));

    // Publish initial mode and precision states after a short delay
    std::thread([this]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std_msgs::msg::String mode_msg;
      mode_msg.data = current_mode_;
      mode_pub_->publish(mode_msg);
      std_msgs::msg::Bool prec_msg;
      prec_msg.data = false;
      right_arm_precision_pub_->publish(prec_msg);
      left_arm_precision_pub_->publish(prec_msg);
    }).detach();

    RCLCPP_INFO(this->get_logger(), "JoyBaseTeleop started.");
  }

private:
  // ── Device parameter declarations (always Logitech 3D Pro) ─────────────────
  void declare_logitech_params()
  {
    // Single Logitech 3D Pro — handle all via base_joy
    this->declare_parameter("base_joy_topic", "/joy");
    this->declare_parameter("aux_joy_topic",  "/joy"); // same topic, filtered by callback

    // Base control through stick X (0) / Y (1) / twist (3)
    this->declare_parameter("axis_x", 0);
    this->declare_parameter("axis_y", 1);
    this->declare_parameter("axis_yaw", 3);       // twist
    this->declare_parameter("invert_x", false);
    this->declare_parameter("invert_y", false);
    this->declare_parameter("invert_yaw", false);

    // Head/elevator via hat switch: axes 4 (hat X) and 5 (hat Y)
    // Hat Y → elevator (z), Hat X → head pan
    this->declare_parameter("axis_z", 5);          // hat Y → elevator
    this->declare_parameter("axis_head_pan", 4);   // hat X → head pan
    this->declare_parameter("invert_z", false);
    this->declare_parameter("invert_head_pan", false);

    // No pitch axis on 3D Pro
    this->declare_parameter("axis_pitch", -1);

    // Throttle on axis 2 (slider, -1..+1 → 0..1 multiplier)
    this->declare_parameter("throttle_enabled", true);
    this->declare_parameter("throttle_axis", 2);

    // Button assignments
    this->declare_parameter("button_precision", 0);    // Trigger → precision toggle
    this->declare_parameter("button_mode_switch", 1);  // Thumb → mode switch
    this->declare_parameter("button_home_base", 2);    // Top button 1 → zero velocity

    // Lower speeds for joystick control
    this->declare_parameter("max_linear_vel", 0.6);
    this->declare_parameter("max_angular_vel", 0.5);
    this->declare_parameter("head_step", 0.03125);

    // Topic and limit parameters referenced by constructor/callbacks
    this->declare_parameter("head_topic",
      "/leader/joystick_controller_left/joint_trajectory");
    this->declare_parameter("joint_state_topic", "/joint_states");
    this->declare_parameter<std::vector<double>>("head_lower_limits",
                                                   {-0.2317, -0.35});
    this->declare_parameter<std::vector<double>>("head_upper_limits",
                                                   {0.6951, 0.35});
  }

  // ── Joint state feedback ──────────────────────────────────────────────────
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state_count_++;
    std::lock_guard<std::mutex> lock(joint_mutex_);

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "head_joint1")
        latest_phys_head_[0] = msg->position[i];
      else if (msg->name[i] == "head_joint2")
        latest_phys_head_[1] = msg->position[i];
    }

    if (!head_initialized_) {
      current_head_pos_[0] = latest_phys_head_[0];
      current_head_pos_[1] = latest_phys_head_[1];
      if (latest_phys_head_[0] != 0.0 || latest_phys_head_[1] != 0.0) {
        head_initialized_ = true;
      }
    } else if (!was_moving_head_) {
      current_head_pos_[0] = latest_phys_head_[0];
      current_head_pos_[1] = latest_phys_head_[1];
    }
  }

  double get_axis(const sensor_msgs::msg::Joy &joy, int index, bool invert) {
    if (index >= 0 && index < static_cast<int>(joy.axes.size())) {
      double val = joy.axes[index];
      if (std::abs(val) < 0.05)
        return 0.0; // deadband
      return invert ? -val : val;
    }
    return 0.0;
  }

  // ── Base joy callback: cmd_vel + Logitech button actions ───────────────────
  void base_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Logitech single-button edge detection (handles precision, home)
    check_logitech_buttons(*msg);

    // Throttle scaling (Logitech 3D Pro)
    double throttle_scale = 1.0;
    bool throttle_enabled = this->get_parameter("throttle_enabled").as_bool();
    if (throttle_enabled) {
      int throttle_axis = this->get_parameter("throttle_axis").as_int();
      if (throttle_axis >= 0 && throttle_axis < static_cast<int>(msg->axes.size())) {
        double raw = msg->axes[throttle_axis];
        throttle_scale = (raw + 1.0) * 0.5; // [-1..+1] → [0..1]
        throttle_scale = std::clamp(throttle_scale, 0.0, 1.0);
      }
    }

    if (current_mode_ != "BASE")
      return;

    double max_linear_vel = this->get_parameter("max_linear_vel").as_double();
    double max_angular_vel = this->get_parameter("max_angular_vel").as_double();

    double x = get_axis(*msg, this->get_parameter("axis_x").as_int(),
                        this->get_parameter("invert_x").as_bool());
    double y = get_axis(*msg, this->get_parameter("axis_y").as_int(),
                        this->get_parameter("invert_y").as_bool());
    double yaw = get_axis(*msg, this->get_parameter("axis_yaw").as_int(),
                          this->get_parameter("invert_yaw").as_bool());

    geometry_msgs::msg::Twist twist;
    // Cubic scaling for finer control at low speeds
    twist.linear.x = (x * x * x) * max_linear_vel * throttle_scale;
    twist.linear.y = (y * y * y) * max_linear_vel * throttle_scale;
    twist.angular.z = (yaw * yaw * yaw) * max_angular_vel * throttle_scale;
    cmd_vel_pub_->publish(twist);
  }

  // ── Logitech 3D Pro auxiliary: head/elevator from hat switch ──────────────
  void aux_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (current_mode_ != "BASE")
      return;

    double head_step = this->get_parameter("head_step").as_double();

    // Hat axes 4 (X, pan) and 5 (Y, elevator/z)
    double elevator = get_axis(*msg, this->get_parameter("axis_z").as_int(),
                                this->get_parameter("invert_z").as_bool());
    double pan = get_axis(*msg, this->get_parameter("axis_head_pan").as_int(),
                           this->get_parameter("invert_head_pan").as_bool());

    bool is_moving = (std::abs(elevator) > 0.01 || std::abs(pan) > 0.01);

    std::lock_guard<std::mutex> lock(joint_mutex_);

    bool publish_needed = false;
    if (is_moving) {
      double pan_cubed = pan * pan * pan;
      double elevator_cubed = elevator * elevator * elevator;
      current_head_pos_[0] =
          std::clamp(current_head_pos_[0] + (elevator_cubed * head_step),
                     head_lower_limits_[0], head_upper_limits_[0]);
      current_head_pos_[1] =
          std::clamp(current_head_pos_[1] + (pan_cubed * head_step),
                     head_lower_limits_[1], head_upper_limits_[1]);
      was_moving_head_ = true;
      publish_needed = true;
    } else if (was_moving_head_) {
      if (hardware_mode_ && !sim_) {
        current_head_pos_[0] = latest_phys_head_[0];
        current_head_pos_[1] = latest_phys_head_[1];
      }
      was_moving_head_ = false;
      publish_needed = true;
    }

    if (publish_needed) {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"head_joint1", "head_joint2"};
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.time_from_start.nanosec = 50000000; // 50 ms
      pt.positions = {current_head_pos_[0], current_head_pos_[1]};
      pt.velocities = {0.0, 0.0};
      traj.points.push_back(pt);
      head_pub_->publish(traj);
    }
  }

  // ── Right SpaceMouse button callback (dedicated subscription) ─────────────
  void right_sm_button_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size() < 1) return;
    double now = this->now().seconds();

    // Button 0 double-click → toggle right arm precision
    {
      bool pressed = msg->buttons[0] > 0;
      if (right_sm_dc_.update(pressed, now)) {
        right_arm_precision_mode_ = !right_arm_precision_mode_;
        RCLCPP_INFO(this->get_logger(), "RIGHT ARM PRECISION: %s",
                    right_arm_precision_mode_ ? "ON" : "OFF");
        std_msgs::msg::Bool msg;
        msg.data = right_arm_precision_mode_;
        right_arm_precision_pub_->publish(msg);
        right_sm_dc_.reset();
      }
    }

    // Button 1 double-click → cycle mode
    if (msg->buttons.size() >= 2) {
      bool pressed = msg->buttons[1] > 0;
      if (right_sm_dc_btn1_.update(pressed, now)) {
        cycle_mode();
        right_sm_dc_btn1_.reset();
      }
    }
  }

  // ── Left SpaceMouse button callback (dedicated subscription) ──────────────
  void left_sm_button_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size() < 1) return;
    double now = this->now().seconds();

    // Button 0 double-click → toggle left arm precision
    {
      bool pressed = msg->buttons[0] > 0;
      if (left_sm_dc_.update(pressed, now)) {
        left_arm_precision_mode_ = !left_arm_precision_mode_;
        RCLCPP_INFO(this->get_logger(), "LEFT ARM PRECISION: %s",
                    left_arm_precision_mode_ ? "ON" : "OFF");
        std_msgs::msg::Bool msg;
        msg.data = left_arm_precision_mode_;
        left_arm_precision_pub_->publish(msg);
        left_sm_dc_.reset();
      }
    }

    // Button 1 double-click → cycle mode
    if (msg->buttons.size() >= 2) {
      bool pressed = msg->buttons[1] > 0;
      if (left_sm_dc_btn1_.update(pressed, now)) {
        cycle_mode();
        left_sm_dc_btn1_.reset();
      }
    }
  }

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr base_joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr aux_joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr right_sm_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr left_sm_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_arm_precision_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_arm_precision_pub_;
  rclcpp::TimerBase::SharedPtr failsafe_timer_;

  std::mutex joint_mutex_;
  std::vector<double> current_head_pos_{0.0, 0.0};
  std::vector<double> latest_phys_head_{0.0, 0.0};
  std::vector<double> head_lower_limits_;
  std::vector<double> head_upper_limits_;
  bool head_initialized_{false};
  bool was_moving_head_{false};
  std::string current_mode_{"BASE"};

  bool right_arm_precision_mode_{false};
  bool left_arm_precision_mode_{false};
  double last_right_arm_precision_toggle_{0.0};
  double last_left_arm_precision_toggle_{0.0};
  int joint_state_count_{0};
  int failsafe_check_cycles_{0};
  bool hardware_mode_{true};
  bool sim_{false};

  // ── SpaceMouse double-click state ─────────────────────────────────────────
  DoubleClickState right_sm_dc_;
  DoubleClickState right_sm_dc_btn1_;
  DoubleClickState left_sm_dc_;
  DoubleClickState left_sm_dc_btn1_;

  // ── Logitech subscriber (listens in all modes) ────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr logitech_sub_;
  sensor_msgs::msg::Joy::SharedPtr logitech_latest_;
  std::mutex logitech_mutex_;

  // ── EE twist publisher (Logitech EE control in LOGITECH mode) ────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ee_twist_pub_;
  rclcpp::TimerBase::SharedPtr logitech_ee_timer_;

  // ── Logitech 3D Pro button edge state ────────────────────────────────────
  bool prev_btn_precision_{false};
  bool prev_btn_mode_{false};
  bool prev_btn_home_base_{false};

  // ── Logitech joystick callback (stores latest data, handles mode button) ──
  void logitech_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(logitech_mutex_);
      logitech_latest_ = msg;
    }

    // Always handle Logitech mode button (thumb = button 1)
    int btn_mode = this->get_parameter("button_mode_switch").as_int();
    if (btn_mode >= 0 && btn_mode < static_cast<int>(msg->buttons.size())) {
      bool pressed = msg->buttons[btn_mode] > 0;
      if (pressed && !prev_btn_mode_) {
        cycle_mode();
      }
      prev_btn_mode_ = pressed;
    }
  }

  // ── Publish EE twist from Logitech when in LOGITECH mode ────────────────
  void publish_logitech_ee() {
    if (current_mode_ != "LOGITECH") return;

    sensor_msgs::msg::Joy::SharedPtr joy;
    {
      std::lock_guard<std::mutex> lock(logitech_mutex_);
      if (!logitech_latest_ || logitech_latest_->axes.size() < 4) return;
      joy = logitech_latest_;
    }

    double x = joy->axes[0];  // stick X
    double y = joy->axes[1];  // stick Y
    double z = joy->axes[3];  // twist → Z
    double pitch = joy->axes[2];  // throttle → pitch rate

    geometry_msgs::msg::Twist cmd;
    const double lin_scale = 0.1;
    const double pitch_scale = 0.5;
    // Cubic scaling for finer low-speed control
    cmd.linear.x = (x * x * x) * lin_scale;
    cmd.linear.y = (y * y * y) * lin_scale;
    cmd.linear.z = (z * z * z) * lin_scale;
    cmd.angular.y = (pitch * pitch * pitch) * pitch_scale;

    ee_twist_pub_->publish(cmd);
  }

  // ── Failsafe ──────────────────────────────────────────────────────────────
  void failsafe_check() {
    if (!hardware_mode_ || sim_) return;

    if (failsafe_check_cycles_ == 0 && joint_state_count_ == 0) {
      return;
    }

    failsafe_check_cycles_++;
    if (failsafe_check_cycles_ > 1) {
      if (joint_state_count_ < 175) {
        RCLCPP_FATAL(this->get_logger(),
          "CRITICAL: Joint states 2.5-sec average dropped to %.1f Hz! "
          "Failsafe triggering shutdown.",
          joint_state_count_ / 2.5);
        rclcpp::shutdown();
      }
    }
    joint_state_count_ = 0;
  }

  // ── Logitech 3D Pro single-button actions ──────────────────────────────────
  void check_logitech_buttons(const sensor_msgs::msg::Joy &joy) {
    // Button 0 (trigger): precision mode toggle
    int btn_precision = this->get_parameter("button_precision").as_int();
    if (btn_precision >= 0 && btn_precision < static_cast<int>(joy.buttons.size())) {
      bool pressed = joy.buttons[btn_precision] > 0;
      if (pressed && !prev_btn_precision_) {
        right_arm_precision_mode_ = !right_arm_precision_mode_;
        RCLCPP_INFO(this->get_logger(), "PRECISION: %s",
                    right_arm_precision_mode_ ? "ON" : "OFF");
        std_msgs::msg::Bool msg;
        msg.data = right_arm_precision_mode_;
        right_arm_precision_pub_->publish(msg);
        // Also toggle left for simplicity with single joystick
        left_arm_precision_mode_ = right_arm_precision_mode_;
        left_arm_precision_pub_->publish(msg);
      }
      prev_btn_precision_ = pressed;
    }

    // Button 2 (top button 1): home base (zero cmd_vel)
    int btn_home = this->get_parameter("button_home_base").as_int();
    if (btn_home >= 0 && btn_home < static_cast<int>(joy.buttons.size())) {
      bool pressed = joy.buttons[btn_home] > 0;
      if (pressed && !prev_btn_home_base_) {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.linear.y = 0.0;
        stop.angular.z = 0.0;
        cmd_vel_pub_->publish(stop);
        RCLCPP_INFO(this->get_logger(), "BASE STOP");
      }
      prev_btn_home_base_ = pressed;
    }
  }

  // ── Cycle mode: BASE → ARM → LOGITECH → BASE ────────────────────────────
  void cycle_mode() {
    if (current_mode_ == "BASE") {
      current_mode_ = "ARM";
    } else if (current_mode_ == "ARM") {
      current_mode_ = "LOGITECH";
    } else {
      current_mode_ = "BASE";
      std::lock_guard<std::mutex> lock(joint_mutex_);
      head_initialized_ = false;
    }

    RCLCPP_INFO(this->get_logger(), "MODE -> %s", current_mode_.c_str());
    std_msgs::msg::String msg;
    msg.data = current_mode_;
    mode_pub_->publish(msg);

    // Reset click and double-click counters so stale events don't re-fire
    right_sm_dc_.reset();
    right_sm_dc_btn1_.reset();
    left_sm_dc_.reset();
    left_sm_dc_btn1_.reset();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyBaseTeleop>());
  rclcpp::shutdown();
  return 0;
}
