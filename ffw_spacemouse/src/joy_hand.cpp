#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class JoyHand : public rclcpp::Node
{
public:
  JoyHand() : Node("joy_hand")
  {
    this->declare_parameter("pos_step", 0.005);
    this->declare_parameter("rot_step", 0.05); // radians
    this->declare_parameter("publish_rate_hz", 100.0);
    this->declare_parameter("trans_sensitivity", 1.5);
    this->declare_parameter("rot_sensitivity", 1.0);

    // Axes mapping for SpaceMouse
    this->declare_parameter("reference_frame", "global"); // "global" or "local"
    this->declare_parameter("axis_x", 1);
    this->declare_parameter("axis_y", 0);
    this->declare_parameter("axis_z", 2);
    this->declare_parameter("axis_roll", 3);
    this->declare_parameter("axis_pitch", 4);
    this->declare_parameter("axis_yaw", 5);

    // Invert flags
    this->declare_parameter("invert_x", false);
    this->declare_parameter("invert_y", false);
    this->declare_parameter("invert_z", false);
    this->declare_parameter("invert_roll", false);
    this->declare_parameter("invert_pitch", false);
    this->declare_parameter("invert_yaw", false);
    this->declare_parameter("target_arm", "arm");
    this->declare_parameter("ee_orientation_slack_deg", 1.0);
    target_arm_ = this->get_parameter("target_arm").as_string();
    ee_orientation_slack_ = this->get_parameter("ee_orientation_slack_deg").as_double() * M_PI / 180.0;

    std::string topic_name = "/spacemouse/" + target_arm_ + "/ee_target_pose";
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyHand::joy_callback, this, _1));

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/teleop_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data == current_mode_)
            return;  // no transition
          bool was_base = (current_mode_ == "BASE");
          current_mode_ = msg->data;
          // BASE→ARM: re-base the mapper into the world frame from the latest
          // achieved pose, so the switch injects no phantom delta (§11).
          if (was_base && current_mode_ == "ARM" && achieved_pose_valid_) {
            ee_goal_ = achieved_pose_map_;
            RCLCPP_INFO(this->get_logger(),
              "Rebased ee_goal_ to achieved pose on ARM switch (%s)", target_arm_.c_str());
          }
      });

    precision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/spacemouse/" + target_arm_ + "/precision_mode", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          precision_mode_ = msg->data;
      });

    // ── Achieved-pose subscription (quest_teleop_plan §11, Task 1) ──
    // World-frame EE pose from the solver, used to re-base the mapper's delta
    // accumulation onto the real pose. No new mutex: the timer and this
    // callback share the single executor thread (like precision_mode_).
    achieved_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ik_solver/achieved_ee_pose_" + target_arm_.substr(0, 1), 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        achieved_pose_map_.translation() << msg->pose.position.x,
            msg->pose.position.y, msg->pose.position.z;
        Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                             msg->pose.orientation.y, msg->pose.orientation.z);
        achieved_pose_map_.linear() = q.normalized().toRotationMatrix();
        if (!achieved_pose_valid_) {
          achieved_pose_valid_ = true;
          // First achieved pose while already in ARM: re-base into the world
          // frame so the mapper starts from the real pose, not Identity.
          if (current_mode_ == "ARM") {
            ee_goal_ = achieved_pose_map_;
            RCLCPP_INFO(this->get_logger(),
              "Rebased ee_goal_ to first achieved pose (%s)", target_arm_.c_str());
          }
        }
      });

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
        // target_arm_ is "left" or "right" (from launch), axis_arm suffix is "_l" or "_r" (from CLI)
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
          // Extract RPY from current ee_goal_ (extrinsic XYZ: Rx*Ry*Rz)
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
      std::bind(&JoyHand::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "SpaceMouse pose mapper started. Publishing to /spacemouse/ee_target_pose");
  }

private:
  // ── EE lock helpers ─────────────────────────────────────────────

  static Eigen::Vector3d extract_rpy(const Eigen::Matrix3d &R) {
    // Extrinsic XYZ Euler: R = Rx(roll) * Ry(pitch) * Rz(yaw)
    double pitch = std::asin(std::clamp(R(0, 2), -1.0, 1.0));
    double yaw = std::atan2(-R(0, 1), R(0, 0));
    double roll = std::atan2(-R(1, 2), R(2, 2));
    return Eigen::Vector3d(roll, pitch, yaw);
  }

  // True when all mapped SpaceMouse axes are near neutral — gates the drift
  // re-base so we never yank the goal out from under an active input (§11).
  bool spacemouse_idle(const sensor_msgs::msg::Joy &joy) {
    for (const std::string axis : {"axis_x", "axis_y", "axis_z",
                                   "axis_roll", "axis_pitch", "axis_yaw"}) {
      int idx = this->get_parameter(axis).as_int();
      if (idx >= 0 && idx < static_cast<int>(joy.axes.size()) &&
          std::abs(joy.axes[idx]) > 0.01)
        return false;
    }
    return true;
  }

  void apply_ee_locks() {
    // Clamp locked roll/yaw axes within slack of their locked centers
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

    // Reconstruct: R = Rx(roll) * Ry(pitch) * Rz(yaw)
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

    // Drift re-base (quest_teleop_plan §11): while in ARM, if the solver's
    // achieved pose has drifted >10cm from the mapper's accumulated goal,
    // snap back to the achieved pose. Gated on SpaceMouse idle (both axes ~0)
    // and Quest teleop inactive, so we never fight the operator.
    if (!quest_active_ && achieved_pose_valid_ && spacemouse_idle(joy)) {
      double dist = (achieved_pose_map_.translation() - ee_goal_.translation()).norm();
      if (dist > 0.10) {
        ee_goal_ = achieved_pose_map_;
        RCLCPP_INFO(this->get_logger(),
          "Drift re-base: snapped ee_goal_ to achieved (%.3f m gap)", dist);
      }
    }

    double pos_step = this->get_parameter("pos_step").as_double();
    double rot_step = this->get_parameter("rot_step").as_double();

    auto get_axis = [&joy](int index, bool invert) -> double {
      if (index >= 0 && index < static_cast<int>(joy.axes.size())) {
        double val = joy.axes[index];
        return invert ? -val : val;
      }
      return 0.0;
    };

    Eigen::Vector3d trans_raw(
      get_axis(this->get_parameter("axis_x").as_int(), this->get_parameter("invert_x").as_bool()),
      get_axis(this->get_parameter("axis_y").as_int(), this->get_parameter("invert_y").as_bool()),
      get_axis(this->get_parameter("axis_z").as_int(), this->get_parameter("invert_z").as_bool())
    );

    double trans_sens = this->get_parameter("trans_sensitivity").as_double();
    trans_raw *= trans_sens;
    double trans_norm = trans_raw.norm();
    if (trans_norm > 1.0) {
      trans_raw /= trans_norm;
      trans_norm = 1.0;
    }
    Eigen::Vector3d trans_scaled = trans_raw * (trans_norm * trans_norm); // Cubic magnitude
    if (precision_mode_) {
      trans_scaled *= 0.1;
    }
    double dx = trans_scaled.x();
    double dy = trans_scaled.y();
    double dz = trans_scaled.z();

    Eigen::Vector3d rot_raw(
      get_axis(this->get_parameter("axis_roll").as_int(), this->get_parameter("invert_roll").as_bool()),
      get_axis(this->get_parameter("axis_pitch").as_int(), this->get_parameter("invert_pitch").as_bool()),
      get_axis(this->get_parameter("axis_yaw").as_int(), this->get_parameter("invert_yaw").as_bool())
    );

    double rot_sens = this->get_parameter("rot_sensitivity").as_double();
    rot_raw *= rot_sens;
    double rot_norm = rot_raw.norm();
    if (rot_norm > 1.0) {
      rot_raw /= rot_norm;
      rot_norm = 1.0;
    }
    Eigen::Vector3d rot_scaled = rot_raw * (rot_norm * rot_norm); // Cubic magnitude
    if (precision_mode_) {
      rot_scaled *= 0.1;
    }
    double drx = rot_scaled.x();
    double dry = rot_scaled.y();
    double drz = rot_scaled.z();

    if (joy.axes.size() >= 3) {
      // Print removed to reduce terminal spam
    }

    std::string ref_frame = this->get_parameter("reference_frame").as_string();

    Eigen::Vector3d trans_delta(dx * pos_step, dy * pos_step, dz * pos_step);

    // Apply translation
    if (ref_frame == "local") {
      ee_goal_.translation() += ee_goal_.linear() * trans_delta;
    } else {
      ee_goal_.translation() += trans_delta;
    }

    // Map straight to angle axis for quaternion
    Eigen::Vector3d rot_axis(drx, dry, drz);
    double angle = rot_axis.norm() * rot_step;

    Eigen::Quaterniond q_delta = Eigen::Quaterniond::Identity();
    if (angle > 1e-6) {
      q_delta = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rot_axis.normalized()));
    }

    // Multiply old quat vs this for the new value
    Eigen::Quaterniond q_old(ee_goal_.linear());
    Eigen::Quaterniond q_new;

    if (ref_frame == "local") {
      q_new = q_old * q_delta;
    } else {
      q_new = q_delta * q_old;
    }

    q_new.normalize();
    ee_goal_.linear() = q_new.toRotationMatrix();

    // Apply EE orientation locks (clamp yaw/roll after integrating spacemouse delta)
    {
      std::lock_guard<std::mutex> lock(ee_lock_mutex_);
      apply_ee_locks();
    }

    publish_pose();
  }

  void publish_pose()
  {
    // Don't publish a world-frame goal until the solver's achieved pose has
    // been seen; before that, ee_goal_ is just Identity and publishing it
    // would snap the arm to the origin (§11).
    if (!achieved_pose_valid_) return;

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

  // ── Subscriptions & publishers ──────────────────────────────────

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr precision_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ee_lock_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr achieved_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string target_arm_;
  std::mutex joy_mutex_;
  sensor_msgs::msg::Joy latest_joy_;
  bool joy_received_ {false};
  std::string current_mode_ {"BASE"};

  // ── World-frame re-base state (quest_teleop_plan §11, Task 1) ──
  Eigen::Isometry3d achieved_pose_map_ {Eigen::Isometry3d::Identity()};
  bool achieved_pose_valid_ {false};
  bool quest_active_ {false};  // set by Quest override (Task 2); disables drift re-base

  Eigen::Isometry3d ee_goal_;
  double command_dt_ {0.01};

  bool precision_mode_ {false};

  // ── EE lock state ───────────────────────────────────────────────
  std::mutex ee_lock_mutex_;
  bool soft_lock_ee_roll_ {false};
  bool soft_lock_ee_yaw_ {false};
  double soft_locked_roll_ {0.0};
  double soft_locked_yaw_ {0.0};
  double ee_orientation_slack_ {0.0};  // set from parameter ee_orientation_slack_deg
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyHand>());
  rclcpp::shutdown();
  return 0;
}
