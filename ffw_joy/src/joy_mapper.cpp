#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

// ─────────────────────────────────────────────────────────────────────────────
// JoyMapper
//
// Translates raw SpaceMouse joystick axes into incremental end-effector pose
// deltas for arm control. Always uses SpaceMouse axis mapping (right/left arm
// SpaceMice); the Logitech 3D Pro controls base/head/elevator via joy_base_teleop.
//
// Default axis mappings:
//   SpaceMouse: x=1, y=0, z=2, roll=4, pitch=3, yaw=5  (roll/pitch
//               intentionally mapped so SpaceMouse pitch-twist = axis 3)
// ─────────────────────────────────────────────────────────────────────────────
class JoyMapper : public rclcpp::Node
{
public:
  JoyMapper() : Node("joy_mapper")
  {
    this->declare_parameter("target_arm", "arm");
    this->declare_parameter("publish_rate_hz", 100.0);
    this->declare_parameter("ee_orientation_slack_deg", 1.0);

    target_arm_ = this->get_parameter("target_arm").as_string();
    ee_orientation_slack_ = this->get_parameter("ee_orientation_slack_deg").as_double() * M_PI / 180.0;

    // Declare SpaceMouse parameters (mapper always controls arm EE via SpaceMouse)
    declare_device_params();

    std::string topic_name = "/spacemouse/" + target_arm_ + "/ee_target_pose";
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyMapper::joy_callback, this, _1));

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/teleop_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          current_mode_ = msg->data;
      });

    precision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/spacemouse/" + target_arm_ + "/precision_mode", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          precision_mode_ = msg->data;
      });

    // Publishers for device button-triggered actions
    // (used when the mapper manages its own precision/mode, e.g. Logitech standalone)
    precision_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/spacemouse/" + target_arm_ + "/precision_mode", 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/teleop_mode", 10);

    // ── EE lock subscription ──────────────────────────────────────
    ee_lock_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/ik_solver/ee_lock", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(ee_lock_mutex_);
        std::string data = msg->data;
        // Parse: "lock roll_l", "lock yaw_l", "unlock roll_r", "toggle yaw_r"
        std::stringstream ss(data);
        std::string action, axis_arm;
        ss >> action >> axis_arm;
        if (action.empty() || axis_arm.empty()) {
          RCLCPP_WARN(this->get_logger(),
            "Invalid ee_lock format: '%s' — expected '<action> <axis>_<arm>'", data.c_str());
          return;
        }

        // Check if this command is for this mapper's arm
        bool matches_my_arm = false;
        if (target_arm_ == "left" && axis_arm.find("_l") != std::string::npos)
          matches_my_arm = true;
        else if (target_arm_ == "right" && axis_arm.find("_r") != std::string::npos)
          matches_my_arm = true;
        if (!matches_my_arm) return;

        // Determine which lock flag to toggle
        bool *lock_flag = nullptr;
        double *locked_value = nullptr;
        if (axis_arm.find("roll") != std::string::npos) {
          lock_flag = &soft_lock_ee_roll_;
          locked_value = &soft_locked_roll_;
        } else if (axis_arm.find("yaw") != std::string::npos) {
          lock_flag = &soft_lock_ee_yaw_;
          locked_value = &soft_locked_yaw_;
        } else {
          RCLCPP_WARN(this->get_logger(), "Unknown ee_lock axis: '%s'", axis_arm.c_str());
          return;
        }

        bool new_state;
        if (action == "lock") new_state = true;
        else if (action == "unlock") new_state = false;
        else if (action == "toggle") new_state = !(*lock_flag);
        else {
          RCLCPP_WARN(this->get_logger(), "Unknown ee_lock action: '%s'", action.c_str());
          return;
        }

        // If engaging lock, capture current EE orientation
        bool was_locked = *lock_flag;
        *lock_flag = new_state;
        if (!was_locked && new_state) {
          const Eigen::Matrix3d &R = ee_goal_.linear();
          double sin_pitch = -R(0, 2);
          double pitch, yaw;
          (void)pitch;
          if (std::abs(sin_pitch) >= 0.9999999) {
            pitch = std::copysign(M_PI / 2.0, sin_pitch);
            yaw = 0.0;
          } else {
            pitch = std::asin(sin_pitch);
            yaw = std::atan2(-R(0, 1), R(0, 0));
          }
          double roll = std::atan2(-R(1, 2), R(2, 2));
          if (lock_flag == &soft_lock_ee_roll_) *locked_value = roll;
          if (lock_flag == &soft_lock_ee_yaw_) *locked_value = yaw;
          RCLCPP_INFO(this->get_logger(), "EE %s locked at %.3f rad for %s",
            (lock_flag == &soft_lock_ee_roll_ ? "roll" : "yaw"), *locked_value, target_arm_.c_str());
        }
      });

    // Initialize pose at origin
    ee_goal_ = Eigen::Isometry3d::Identity();

    const double rate = std::max(1.0, this->get_parameter("publish_rate_hz").as_double());
    command_dt_ = 1.0 / rate;
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(command_dt_)),
      std::bind(&JoyMapper::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "JoyMapper started. Publishing to %s", topic_name.c_str());
  }

private:
  // ── Declare SpaceMouse parameters ──────────────────────────────────────────
  void declare_device_params()
  {
    // Common inversion flags
    this->declare_parameter("reference_frame", "global");
    this->declare_parameter("invert_x", false);
    this->declare_parameter("invert_y", false);
    this->declare_parameter("invert_z", false);
    this->declare_parameter("invert_roll", false);
    this->declare_parameter("invert_pitch", false);
    this->declare_parameter("invert_yaw", false);

    // SpaceMouse: 6-axis controller
    this->declare_parameter("axis_x", 1);
    this->declare_parameter("axis_y", 0);
    this->declare_parameter("axis_z", 2);
    this->declare_parameter("axis_roll", 4);
    this->declare_parameter("axis_pitch", 3);
    this->declare_parameter("axis_yaw", 5);
    this->declare_parameter("throttle_axis", -1);
    this->declare_parameter("throttle_enabled", false);
    this->declare_parameter("hat_z_enabled", false);
    this->declare_parameter("button_precision", -1);
    this->declare_parameter("button_mode_switch", -1);
    this->declare_parameter("button_home", -1);

    // Step sizes and sensitivities for SpaceMouse
    this->declare_parameter("pos_step", 0.005);
    this->declare_parameter("rot_step", 0.025);
    this->declare_parameter("trans_sensitivity", 1.5);
    this->declare_parameter("rot_sensitivity", 1.0);
  }

  // ── EE lock helpers ─────────────────────────────────────────────

  static Eigen::Vector3d extract_rpy(const Eigen::Matrix3d &R) {
    // Extrinsic XYZ Euler: R = Rx(roll) * Ry(pitch) * Rz(yaw)
    double pitch = std::asin(std::clamp(R(0, 2), -1.0, 1.0));
    double yaw = std::atan2(-R(0, 1), R(0, 0));
    double roll = std::atan2(-R(1, 2), R(2, 2));
    return Eigen::Vector3d(roll, pitch, yaw);
  }

  void apply_ee_locks() {
    if (!soft_lock_ee_roll_ && !soft_lock_ee_yaw_)
      return;

    Eigen::Vector3d rpy = extract_rpy(ee_goal_.linear());
    double roll = rpy.x();
    double pitch = rpy.y();
    double yaw = rpy.z();

    if (soft_lock_ee_roll_)
      roll = std::clamp(roll, soft_locked_roll_ - ee_orientation_slack_,
                              soft_locked_roll_ + ee_orientation_slack_);
    if (soft_lock_ee_yaw_)
      yaw = std::clamp(yaw, soft_locked_yaw_ - ee_orientation_slack_,
                             soft_locked_yaw_ + ee_orientation_slack_);

    Eigen::Matrix3d Rx = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    ee_goal_.linear() = Rx * Ry * Rz;
  }

  // ── Callbacks ───────────────────────────────────────────────────

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    latest_joy_ = *msg;
    joy_received_ = true;

    // Handle button-based actions (Logitech 3D Pro buttons)
    // These are edge-triggered, so process them here
    process_buttons(*msg);
  }

  void process_buttons(const sensor_msgs::msg::Joy &joy)
  {
    // Button edge detection for Logitech 3D Pro functions
    bool any_action = false;
    (void)any_action;

    // Precision mode toggle
    int btn_precision = this->get_parameter("button_precision").as_int();
    if (btn_precision >= 0 && btn_precision < static_cast<int>(joy.buttons.size())) {
      bool pressed = joy.buttons[btn_precision] > 0;
      if (pressed && !prev_btn_precision_) {
        // Toggle precision mode locally via a publisher or just signal
        // The main precision mode is handled via the /spacemouse/<arm>/precision_mode topic
        // So we publish to that topic on button press
        precision_mode_ = !precision_mode_;
        std_msgs::msg::Bool msg;
        msg.data = precision_mode_;
        precision_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "%s precision: %s",
          target_arm_.c_str(), precision_mode_ ? "ON" : "OFF");
      }
      prev_btn_precision_ = pressed;
    }

    // Mode switch (BASE ↔ ARM) — publish to /teleop_mode
    int btn_mode = this->get_parameter("button_mode_switch").as_int();
    if (btn_mode >= 0 && btn_mode < static_cast<int>(joy.buttons.size())) {
      bool pressed = joy.buttons[btn_mode] > 0;
      if (pressed && !prev_btn_mode_) {
        current_mode_ = (current_mode_ == "BASE") ? "ARM" : "BASE";
        std_msgs::msg::String msg;
        msg.data = current_mode_;
        mode_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Mode switch → %s", current_mode_.c_str());
      }
      prev_btn_mode_ = pressed;
    }

    // Home / reset pose
    int btn_home = this->get_parameter("button_home").as_int();
    if (btn_home >= 0 && btn_home < static_cast<int>(joy.buttons.size())) {
      bool pressed = joy.buttons[btn_home] > 0;
      if (pressed && !prev_btn_home_) {
        ee_goal_ = Eigen::Isometry3d::Identity();
        RCLCPP_INFO(this->get_logger(), "EE pose reset to origin for %s", target_arm_.c_str());
      }
      prev_btn_home_ = pressed;
    }
  }

  void control_timer_callback()
  {
    if (current_mode_ == "BASE") return;

    sensor_msgs::msg::Joy joy;
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      if (!joy_received_) return;
      joy = latest_joy_;
    }

    double pos_step = this->get_parameter("pos_step").as_double();
    double rot_step = this->get_parameter("rot_step").as_double();
    bool throttle_enabled = this->get_parameter("throttle_enabled").as_bool();
    int throttle_axis = this->get_parameter("throttle_axis").as_int();
    bool hat_z_enabled = this->get_parameter("hat_z_enabled").as_bool();

    // ── Throttle scaling (Logitech 3D Pro) ──────────────────────
    double throttle_scale = 1.0;
    if (throttle_enabled && throttle_axis >= 0 &&
        throttle_axis < static_cast<int>(joy.axes.size())) {
      // Logitech throttle: -1 (idle) to +1 (full). Remap to 0…1 range.
      double raw = joy.axes[throttle_axis];
      throttle_scale = (raw + 1.0) * 0.5; // [0..1]
      throttle_scale = std::clamp(throttle_scale, 0.0, 1.0);
    }

    auto get_axis = [&joy](int index, bool invert) -> double {
      if (index >= 0 && index < static_cast<int>(joy.axes.size())) {
        double val = joy.axes[index];
        return invert ? -val : val;
      }
      return 0.0;
    };

    // ── Translation ──────────────────────────────────────────────
    double raw_x = get_axis(this->get_parameter("axis_x").as_int(),
                            this->get_parameter("invert_x").as_bool());
    double raw_y = get_axis(this->get_parameter("axis_y").as_int(),
                            this->get_parameter("invert_y").as_bool());
    double raw_z = 0.0;

    // Z can come from a dedicated axis OR from hat switch up/down
    bool z_from_hat = false;
    if (hat_z_enabled && joy.axes.size() >= 6) {
      // Hat Y (axis 5) → up/down discrete steps for Z
      double hat_y = joy.axes[5];
      if (std::abs(hat_y) > 0.5) { // hat is pressed
        raw_z = hat_y;              // +1 up, -1 down (or vice versa with invert)
        z_from_hat = true;
      }
    }
    if (!z_from_hat) {
      raw_z = get_axis(this->get_parameter("axis_z").as_int(),
                       this->get_parameter("invert_z").as_bool());
    }

    Eigen::Vector3d trans_raw(raw_x, raw_y, raw_z);

    double trans_sens = this->get_parameter("trans_sensitivity").as_double();
    trans_raw *= trans_sens;
    double trans_norm = trans_raw.norm();
    if (trans_norm > 1.0) {
      trans_raw /= trans_norm;
      trans_norm = 1.0;
    }
    Eigen::Vector3d trans_scaled = trans_raw * (trans_norm * trans_norm);
    if (precision_mode_) {
      trans_scaled *= 0.1;
    }
    // Apply throttle scaling
    trans_scaled *= throttle_scale;

    double dx = trans_scaled.x();
    double dy = trans_scaled.y();
    double dz = trans_scaled.z();

    // ── Rotation ─────────────────────────────────────────────────
    double raw_roll = get_axis(this->get_parameter("axis_roll").as_int(),
                               this->get_parameter("invert_roll").as_bool());
    double raw_pitch = get_axis(this->get_parameter("axis_pitch").as_int(),
                                this->get_parameter("invert_pitch").as_bool());
    double raw_yaw = get_axis(this->get_parameter("axis_yaw").as_int(),
                              this->get_parameter("invert_yaw").as_bool());

    Eigen::Vector3d rot_raw(raw_roll, raw_pitch, raw_yaw);

    double rot_sens = this->get_parameter("rot_sensitivity").as_double();
    rot_raw *= rot_sens;
    double rot_norm = rot_raw.norm();
    if (rot_norm > 1.0) {
      rot_raw /= rot_norm;
      rot_norm = 1.0;
    }
    Eigen::Vector3d rot_scaled = rot_raw * (rot_norm * rot_norm);
    if (precision_mode_) {
      rot_scaled *= 0.1;
    }
    rot_scaled *= throttle_scale;

    double drx = rot_scaled.x();
    double dry = rot_scaled.y();
    double drz = rot_scaled.z();

    // ── Apply deltas ─────────────────────────────────────────────
    std::string ref_frame = this->get_parameter("reference_frame").as_string();

    Eigen::Vector3d trans_delta(dx * pos_step, dy * pos_step, dz * pos_step);

    if (ref_frame == "local") {
      ee_goal_.translation() += ee_goal_.linear() * trans_delta;
    } else {
      ee_goal_.translation() += trans_delta;
    }

    Eigen::Vector3d rot_axis(drx, dry, drz);
    double angle = rot_axis.norm() * rot_step;

    Eigen::Quaterniond q_delta = Eigen::Quaterniond::Identity();
    if (angle > 1e-6) {
      q_delta = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rot_axis.normalized()));
    }

    Eigen::Quaterniond q_old(ee_goal_.linear());
    Eigen::Quaterniond q_new;

    if (ref_frame == "local") {
      q_new = q_old * q_delta;
    } else {
      q_new = q_delta * q_old;
    }

    q_new.normalize();
    ee_goal_.linear() = q_new.toRotationMatrix();

    // Apply EE orientation locks
    {
      std::lock_guard<std::mutex> lock(ee_lock_mutex_);
      apply_ee_locks();
    }

    publish_pose();
  }

  void publish_pose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = ee_goal_.translation().x();
    msg.pose.position.y = ee_goal_.translation().y();
    msg.pose.position.z = ee_goal_.translation().z();
    Eigen::Quaterniond q(ee_goal_.linear());
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pose_pub_->publish(msg);
  }

  // ── Members ────────────────────────────────────────────────────

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr precision_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ee_lock_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr precision_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string target_arm_;
  std::mutex joy_mutex_;
  sensor_msgs::msg::Joy latest_joy_;
  bool joy_received_ {false};
  std::string current_mode_ {"BASE"};

  Eigen::Isometry3d ee_goal_;
  double command_dt_ {0.01};

  bool precision_mode_ {false};

  // ── Button edge state (Logitech 3D Pro) ────────────────────────
  bool prev_btn_precision_ {false};
  bool prev_btn_mode_ {false};
  bool prev_btn_home_ {false};

  // ── EE lock state ──────────────────────────────────────────────
  std::mutex ee_lock_mutex_;
  bool soft_lock_ee_roll_ {false};
  bool soft_lock_ee_yaw_ {false};
  double soft_locked_roll_ {0.0};
  double soft_locked_yaw_ {0.0};
  double ee_orientation_slack_ {0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyMapper>());
  rclcpp::shutdown();
  return 0;
}
