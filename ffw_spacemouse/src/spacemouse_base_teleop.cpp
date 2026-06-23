#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

struct ButtonClickState {
    double last_press_time_sec = 0.0;
    int count = 0;
    bool prev_pressed = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// SpaceMouseBaseTeleop
//
// Physical assignment (BASE mode):
//   base_joy  = RIGHT SpaceMouse → cmd_vel (swerve base)
//   aux_joy   = LEFT  SpaceMouse → elevator (lift) + head pan/tilt
//
// ARM mode (handled by spacemouse_mapper / IK solver):
//   RIGHT SpaceMouse → right arm   |   LEFT SpaceMouse → left arm
//
// Precision mode (double-click both buttons on same mouse):
//   base_joy (right mouse) → toggles RIGHT arm precision mode
//   aux_joy  (left mouse)  → toggles LEFT  arm precision mode
//
// Mode switch (double-click both buttons on BOTH mice simultaneously):
//   Switches between BASE and ARM modes.
// ─────────────────────────────────────────────────────────────────────────────
class SpaceMouseBaseTeleop : public rclcpp::Node
{
public:
  SpaceMouseBaseTeleop() : Node("spacemouse_base_teleop")
  {
    // ── Base joy parameters (RIGHT SpaceMouse → swerve base) ─────────────────
    this->declare_parameter("base_joy_topic", "/right/joy");
    this->declare_parameter("max_linear_vel", 1.0);
    this->declare_parameter("max_angular_vel", 1.5);
    this->declare_parameter("axis_x",   1);
    this->declare_parameter("axis_y",   0);
    this->declare_parameter("axis_yaw", 5);
    this->declare_parameter("invert_x",   false);
    this->declare_parameter("invert_y",   false);
    this->declare_parameter("invert_yaw", false);

    // ── Aux joy parameters (LEFT SpaceMouse → elevator + head) ───────────────
    this->declare_parameter("aux_joy_topic", "/left/joy");
    this->declare_parameter("axis_z",        2);
    this->declare_parameter("axis_pitch",    3);
    this->declare_parameter("axis_head_pan", 5);
    this->declare_parameter("invert_z",        false);
    this->declare_parameter("invert_pitch",    false);
    this->declare_parameter("invert_head_pan", false);
    this->declare_parameter("head_step", 0.05);

    // ── Joint topics ──────────────────────────────────────────────────────────
    this->declare_parameter("head_topic",        "/leader/joystick_controller_left/joint_trajectory");
    this->declare_parameter("joint_state_topic", "/joint_states");

    // ── Joint limits ──────────────────────────────────────────────────────────
    this->declare_parameter<std::vector<double>>("head_lower_limits", {-0.2317, -0.35});
    this->declare_parameter<std::vector<double>>("head_upper_limits", {0.6951,  0.35});

    head_lower_limits_ = this->get_parameter("head_lower_limits").as_double_array();
    head_upper_limits_ = this->get_parameter("head_upper_limits").as_double_array();

    // ── Subscribers ───────────────────────────────────────────────────────────
    base_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      this->get_parameter("base_joy_topic").as_string(), 10,
      std::bind(&SpaceMouseBaseTeleop::base_joy_callback, this, _1));

    aux_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      this->get_parameter("aux_joy_topic").as_string(), 10,
      std::bind(&SpaceMouseBaseTeleop::aux_joy_callback, this, _1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      this->get_parameter("joint_state_topic").as_string(), 20,
      std::bind(&SpaceMouseBaseTeleop::joint_state_callback, this, _1));

    // ── Publishers ────────────────────────────────────────────────────────────
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    head_pub_    = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("head_topic").as_string(), 10);
    mode_pub_    = this->create_publisher<std_msgs::msg::String>("/teleop_mode", 10);

    // Precision: base mouse (right) → right arm, aux mouse (left) → left arm
    right_arm_precision_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/spacemouse/right/precision_mode", 10);
    left_arm_precision_pub_  = this->create_publisher<std_msgs::msg::Bool>(
      "/spacemouse/left/precision_mode", 10);

    // ── Initial state ─────────────────────────────────────────────────────────
    current_head_pos_  = {0.0, 0.0};
    current_mode_      = "BASE";

    // Failsafe timer (checks every 1.0 seconds)
    failsafe_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SpaceMouseBaseTeleop::failsafe_check, this));

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

    RCLCPP_INFO(this->get_logger(),
      "SpaceMouse Base Teleop started. base_joy=%s (RIGHT mouse, base), aux_joy=%s (LEFT mouse, head)",
      this->get_parameter("base_joy_topic").as_string().c_str(),
      this->get_parameter("aux_joy_topic").as_string().c_str());
  }

private:
  // ── Joint state feedback ────────────────────────────────────────────────────
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_state_count_++;
    std::lock_guard<std::mutex> lock(joint_mutex_);
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if      (msg->name[i] == "head_joint1") latest_phys_head_[0] = msg->position[i];
      else if (msg->name[i] == "head_joint2") latest_phys_head_[1] = msg->position[i];
    }
    
    if (!head_initialized_) {
        current_head_pos_[0] = latest_phys_head_[0];
        current_head_pos_[1] = latest_phys_head_[1];
        if (latest_phys_head_[0] != 0.0 || latest_phys_head_[1] != 0.0) {
            head_initialized_ = true;
        }
    }
  }

  double get_axis(const sensor_msgs::msg::Joy& joy, int index, bool invert) {
    if (index >= 0 && index < static_cast<int>(joy.axes.size())) {
      double val = joy.axes[index];
      if (std::abs(val) < 0.05) return 0.0;  // deadband
      return invert ? -val : val;
    }
    return 0.0;
  }

  // ── RIGHT SpaceMouse: base movement (cmd_vel) ───────────────────────────────
  void base_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(button_mutex_);
      for (size_t i = 0; i < msg->buttons.size() && i < 2; ++i) {
        bool pressed = msg->buttons[i] > 0;
        if (pressed && !base_clicks_[i].prev_pressed) {
          double now = this->now().seconds();
          base_clicks_[i].count = ((now - base_clicks_[i].last_press_time_sec) > 0.5) ? 1
                                                                                        : base_clicks_[i].count + 1;
          base_clicks_[i].last_press_time_sec = now;
        }
        base_clicks_[i].prev_pressed = pressed;
      }
      check_button_double_clicks();
    }

    if (current_mode_ != "BASE") return;

    double max_linear_vel  = this->get_parameter("max_linear_vel").as_double();
    double max_angular_vel = this->get_parameter("max_angular_vel").as_double();

    double x   = get_axis(*msg, this->get_parameter("axis_x").as_int(),   this->get_parameter("invert_x").as_bool());
    double y   = get_axis(*msg, this->get_parameter("axis_y").as_int(),   this->get_parameter("invert_y").as_bool());
    double yaw = get_axis(*msg, this->get_parameter("axis_yaw").as_int(), this->get_parameter("invert_yaw").as_bool());

    geometry_msgs::msg::Twist twist;
    // Cubic scaling for finer control at low speeds
    twist.linear.x  = (x   * x   * x)   * max_linear_vel;
    twist.linear.y  = (y   * y   * y)   * max_linear_vel;
    twist.angular.z = (yaw * yaw * yaw) * max_angular_vel;
    cmd_vel_pub_->publish(twist);
  }

  // ── LEFT SpaceMouse: head pan/tilt ─────────────────────────────────────────
  void aux_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(button_mutex_);
      for (size_t i = 0; i < msg->buttons.size() && i < 2; ++i) {
        bool pressed = msg->buttons[i] > 0;
        if (pressed && !aux_clicks_[i].prev_pressed) {
          double now = this->now().seconds();
          aux_clicks_[i].count = ((now - aux_clicks_[i].last_press_time_sec) > 0.5) ? 1
                                                                                      : aux_clicks_[i].count + 1;
          aux_clicks_[i].last_press_time_sec = now;
        }
        aux_clicks_[i].prev_pressed = pressed;
      }
      check_button_double_clicks();
    }

    if (current_mode_ != "BASE") return;

    double head_step = this->get_parameter("head_step").as_double();
    double pitch = get_axis(*msg, this->get_parameter("axis_pitch").as_int(),    this->get_parameter("invert_pitch").as_bool());
    double pan   = get_axis(*msg, this->get_parameter("axis_head_pan").as_int(), this->get_parameter("invert_head_pan").as_bool());

    bool is_moving = (std::abs(pitch) > 0.01 || std::abs(pan) > 0.01);

    std::lock_guard<std::mutex> lock(joint_mutex_);
    
    if (is_moving) {
        current_head_pos_[0] = std::clamp(current_head_pos_[0] + (pan   * head_step), head_lower_limits_[0], head_upper_limits_[0]);
        current_head_pos_[1] = std::clamp(current_head_pos_[1] + (pitch * head_step), head_lower_limits_[1], head_upper_limits_[1]);
        was_moving_head_ = true;
    } else if (was_moving_head_) {
        // Dead stop: snap target to actual physical position to instantly zero out tracking error
        current_head_pos_[0] = latest_phys_head_[0];
        current_head_pos_[1] = latest_phys_head_[1];
        was_moving_head_ = false;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"head_joint1", "head_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.time_from_start.nanosec = 50000000;  // 50 ms
    pt.positions  = {current_head_pos_[0], current_head_pos_[1]};
    pt.velocities = {0.0, 0.0};
    traj.points.push_back(pt);
    head_pub_->publish(traj);
  }

  // ── Members ─────────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr        base_joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr        aux_joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   right_arm_precision_pub_;  // base (right) mouse
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   left_arm_precision_pub_;   // aux  (left)  mouse
  rclcpp::TimerBase::SharedPtr failsafe_timer_;

  std::mutex joint_mutex_;
  std::vector<double> current_head_pos_ {0.0, 0.0};
  std::vector<double> latest_phys_head_ {0.0, 0.0};
  bool head_initialized_ {false};
  bool was_moving_head_ {false};
  std::string current_mode_ {"BASE"};

  std::vector<double> head_lower_limits_;
  std::vector<double> head_upper_limits_;

  std::mutex button_mutex_;
  std::vector<ButtonClickState> base_clicks_{2};  // RIGHT mouse (base movement)
  std::vector<ButtonClickState> aux_clicks_{2};   // LEFT  mouse (head)

  bool right_arm_precision_mode_ {false};
  bool left_arm_precision_mode_  {false};
  double last_right_arm_precision_toggle_ {0.0};
  double last_left_arm_precision_toggle_  {0.0};
  int joint_state_count_    {0};
  int failsafe_check_cycles_{0};

  // ── Failsafe ─────────────────────────────────────────────────────────────────
  void failsafe_check() {
    if (failsafe_check_cycles_ == 0 && joint_state_count_ == 0) {
      // Suspend failsafe until the first /joint_states message is received.
      // This allows the system to take its time booting up without crashing.
      return;
    }
    
    failsafe_check_cycles_++;
    if (failsafe_check_cycles_ > 3) {  // allow 3s startup after first message
      if (joint_state_count_ < 70) {
        RCLCPP_FATAL(this->get_logger(),
          "CRITICAL: Joint states Hz dropped to %d! Failsafe triggering shutdown.", joint_state_count_);
        rclcpp::shutdown();
      }
    }
    joint_state_count_ = 0;
  }

  // ── Button double-click logic ─────────────────────────────────────────────
  // All-four-buttons: switch BASE ↔ ARM
  // base mouse only: toggle RIGHT arm precision
  // aux  mouse only: toggle LEFT  arm precision
  void check_button_double_clicks() {
    double now = this->now().seconds();
    auto is_dc = [now](const ButtonClickState& s) {
      return s.count >= 2 && (now - s.last_press_time_sec) < 1.0;
    };

    bool base_dc = is_dc(base_clicks_[0]) && is_dc(base_clicks_[1]);
    bool aux_dc  = is_dc(aux_clicks_[0])  && is_dc(aux_clicks_[1]);

    if (base_dc && aux_dc) {
      // All four buttons → mode switch
      current_mode_ = (current_mode_ == "BASE") ? "ARM" : "BASE";
      if (current_mode_ == "BASE") {
          std::lock_guard<std::mutex> lock(joint_mutex_);
          head_initialized_ = false; // Re-sync head with real robot on mode switch
      }
      
      RCLCPP_INFO(this->get_logger(), "ALL FOUR BUTTONS: mode → %s", current_mode_.c_str());
      std_msgs::msg::String msg;
      msg.data = current_mode_;
      mode_pub_->publish(msg);
      base_clicks_[0].count = base_clicks_[1].count = 0;
      aux_clicks_[0].count  = aux_clicks_[1].count  = 0;
    }
    else if (base_dc) {
      // Only check for aux activity within 0.25s to avoid cross-interference
      bool aux_active = (now - aux_clicks_[0].last_press_time_sec < 0.25) ||
                        (now - aux_clicks_[1].last_press_time_sec < 0.25);
      if (!aux_active && (now - last_right_arm_precision_toggle_) > 0.5) {
        right_arm_precision_mode_ = !right_arm_precision_mode_;
        last_right_arm_precision_toggle_ = now;
        RCLCPP_INFO(this->get_logger(), "RIGHT ARM PRECISION: %s", right_arm_precision_mode_ ? "ON" : "OFF");
        std_msgs::msg::Bool msg;
        msg.data = right_arm_precision_mode_;
        right_arm_precision_pub_->publish(msg);
        base_clicks_[0].count = base_clicks_[1].count = 0;
      }
    }
    else if (aux_dc) {
      bool base_active = (now - base_clicks_[0].last_press_time_sec < 0.25) ||
                         (now - base_clicks_[1].last_press_time_sec < 0.25);
      if (!base_active && (now - last_left_arm_precision_toggle_) > 0.5) {
        left_arm_precision_mode_ = !left_arm_precision_mode_;
        last_left_arm_precision_toggle_ = now;
        RCLCPP_INFO(this->get_logger(), "LEFT ARM PRECISION: %s", left_arm_precision_mode_ ? "ON" : "OFF");
        std_msgs::msg::Bool msg;
        msg.data = left_arm_precision_mode_;
        left_arm_precision_pub_->publish(msg);
        aux_clicks_[0].count = aux_clicks_[1].count = 0;
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpaceMouseBaseTeleop>());
  rclcpp::shutdown();
  return 0;
}
