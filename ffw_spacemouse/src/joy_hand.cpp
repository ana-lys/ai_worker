#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
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

    // ── Quest override parameters (quest_teleop_plan §7, Task 2) ──
    this->declare_parameter("quest_state_topic", "/quest_state");
    this->declare_parameter("quest_achieved_pose_topic_l", "/ik_solver/achieved_ee_pose_l");
    this->declare_parameter("quest_achieved_pose_topic_r", "/ik_solver/achieved_ee_pose_r");
    this->declare_parameter("quest_active_topic_prefix", "/quest");
    this->declare_parameter("quest_publish_notifications", true);   // only ONE instance fires
    this->declare_parameter("quest_notification_topic", "/ffw_control/notification");
    this->declare_parameter("quest_hold_engage_s", 2.0);
    this->declare_parameter("quest_release_min", 0.1);   // all 4 floats < this → count arms (§4)
    this->declare_parameter("quest_thumb_analog_offset", 7);    // grip, within arm block (§3)
    this->declare_parameter("quest_trigger_analog_offset", 6);  // trigger, within arm block (§3)
    this->declare_parameter("quest_thumb_engage_min", 0.5);     // thumb >= 0.5 keeps mode alive
    this->declare_parameter("quest_trigger_engage_min", 0.5);
    this->declare_parameter("quest_w_slow", 0.02);
    this->declare_parameter("quest_w_fast", 0.15);
    this->declare_parameter("quest_approach_pos_m", 0.05);
    this->declare_parameter("quest_approach_ang_rad", 0.10);
    this->declare_parameter("quest_timeout_s", 0.5);        // no /quest_state → auto-return
    this->declare_parameter("quest_achieved_max_age_s", 0.5); // engage re-base: achieved age limit
    this->declare_parameter("quest_hold_hysteresis", 0.05); // keep-alive/early-release band (§4)
    this->declare_parameter("quest_no_progress_s", 5.0);    // APPROACH stuck guard (§10)
    this->declare_parameter("quest_frame_rot_rpy", std::vector<double>{0.0, 0.0, 0.0}); // optional alignment
    this->declare_parameter("quest_pos_scale", 1.0);

    quest_state_topic_ = this->get_parameter("quest_state_topic").as_string();
    quest_active_topic_ = this->get_parameter("quest_active_topic_prefix").as_string() +
                          "/" + target_arm_ + "/active";
    quest_notification_topic_ = this->get_parameter("quest_notification_topic").as_string();
    quest_publish_notifications_ = this->get_parameter("quest_publish_notifications").as_bool();
    quest_hold_engage_s_ = this->get_parameter("quest_hold_engage_s").as_double();
    quest_release_min_ = this->get_parameter("quest_release_min").as_double();
    quest_thumb_analog_offset_ = this->get_parameter("quest_thumb_analog_offset").as_int();
    quest_trigger_analog_offset_ = this->get_parameter("quest_trigger_analog_offset").as_int();
    quest_thumb_engage_min_ = this->get_parameter("quest_thumb_engage_min").as_double();
    quest_trigger_engage_min_ = this->get_parameter("quest_trigger_engage_min").as_double();
    quest_w_slow_ = this->get_parameter("quest_w_slow").as_double();
    quest_w_fast_ = this->get_parameter("quest_w_fast").as_double();
    quest_approach_pos_m_ = this->get_parameter("quest_approach_pos_m").as_double();
    quest_approach_ang_rad_ = this->get_parameter("quest_approach_ang_rad").as_double();
    quest_timeout_s_ = this->get_parameter("quest_timeout_s").as_double();
    quest_achieved_max_age_s_ = this->get_parameter("quest_achieved_max_age_s").as_double();
    quest_hold_hysteresis_ = this->get_parameter("quest_hold_hysteresis").as_double();
    quest_no_progress_s_ = this->get_parameter("quest_no_progress_s").as_double();
    quest_pos_scale_ = this->get_parameter("quest_pos_scale").as_double();

    const std::vector<double> frame_rpy =
        this->get_parameter("quest_frame_rot_rpy").as_double_array();
    if (frame_rpy.size() >= 3) {
      quest_frame_rot_.linear() =
          Eigen::AngleAxisd(frame_rpy[0], Eigen::Vector3d::UnitX()).toRotationMatrix() *
          Eigen::AngleAxisd(frame_rpy[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
          Eigen::AngleAxisd(frame_rpy[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    std::string topic_name = "/spacemouse/" + target_arm_ + "/ee_target_pose";
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyHand::joy_callback, this, _1));

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/teleop_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data == current_mode_)
            return;  // no transition
          if (current_mode_ == "ARM" && msg->data != "ARM") {
            // Emergency stop (quest_teleop_plan §4 "Abort", §7): stop listening
            // to the quest, freeze ee_goal_ where it is, reset to the pre-count
            // state. Lives in the callback, not the timer, so it executes even
            // though the timer early-returns in BASE.
            enter_quest_state(QuestState::SM_CONTROL);
            engage_armed_ = false;
            hold_started_ = false;
          } else if (msg->data == "ARM" && current_mode_ != "ARM") {
            // BASE→ARM: re-base the mapper into the world frame from the latest
            // achieved pose, so the switch injects no phantom delta (§11).
            if (achieved_pose_valid_) {
              ee_goal_ = achieved_pose_map_;
              RCLCPP_INFO(this->get_logger(),
                "Rebased ee_goal_ to achieved pose on ARM switch (%s)", target_arm_.c_str());
            }
          }
          current_mode_ = msg->data;
      });

    precision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/spacemouse/" + target_arm_ + "/precision_mode", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          precision_mode_ = msg->data;
      });

    // ── Achieved-pose subscription (quest_teleop_plan §5/§11, Task 1) ──
    // World-frame EE pose from the solver, used to re-base the mapper's delta
    // accumulation onto the real pose and to anchor the Quest engage. No new
    // mutex: the timer and this callback share the single executor thread
    // (like precision_mode_).
    const std::string achieved_topic = (target_arm_ == "right")
        ? this->get_parameter("quest_achieved_pose_topic_r").as_string()
        : this->get_parameter("quest_achieved_pose_topic_l").as_string();
    achieved_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      achieved_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        achieved_pose_map_.translation() << msg->pose.position.x,
            msg->pose.position.y, msg->pose.position.z;
        Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                             msg->pose.orientation.y, msg->pose.orientation.z);
        achieved_pose_map_.linear() = q.normalized().toRotationMatrix();
        // Mapper's own receive time — freshness for the engage re-base is
        // measured from this, NOT header.stamp (§5).
        achieved_pose_received_time_ = this->now();
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

    // ── Quest subscriptions & publishers (quest_teleop_plan §7, Task 2) ──
    quest_state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      quest_state_topic_, 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // §3 flat layout: [left block 0-15][right block 16-31][head pose 32-37].
        quest_data_ = msg->data;          // cache (single-threaded executor → no mutex)
        quest_data_valid_ = true;
        quest_last_msg_time_ = this->now();   // §9 arrival watchdog
      });

    quest_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(quest_active_topic_, 10);
    quest_notification_pub_ =
      this->create_publisher<std_msgs::msg::String>(quest_notification_topic_, 10);

    // Initialize pose at origin
    ee_goal_ = Eigen::Isometry3d::Identity();

    const double rate = std::max(1.0, this->get_parameter("publish_rate_hz").as_double());
    command_dt_ = 1.0 / rate;
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(command_dt_)),
      std::bind(&JoyHand::control_timer_callback, this));

    quest_last_msg_time_ = this->now();   // watchdog baseline before first message

    RCLCPP_INFO(this->get_logger(), "SpaceMouse pose mapper started. Publishing to /spacemouse/ee_target_pose");
    RCLCPP_INFO(this->get_logger(),
      "Quest override ready (%s): active on %s", target_arm_.c_str(),
      quest_active_topic_.c_str());
  }

private:
  // ── /quest_state layout (quest_to_ros2.py, quest_teleop_plan §3) ──
  // Each controller block: [0-5] pose (x,y,z,roll,pitch,yaw), [6] trigger,
  // [7] grip (thumb), [8-9] stick, [10-15] button floats. Right block = +16.
  static constexpr size_t QUEST_RIGHT_OFFSET = 16;

  // Quest override state machine (quest_teleop_plan §4-§10, Task 2).
  // Declared before the quest helpers below: a member function's *parameter
  // list* is not a complete-class context, so the enumerator must be visible
  // at the point of the helper declarations, not just in their bodies.
  enum class QuestState { SM_CONTROL, QUEST_APPROACH, QUEST_READY, QUEST_TRACK };

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

  // ── Quest override helpers (quest_teleop_plan §4-§10, Task 2) ──

  // Analog float for one hand, offset within that hand's controller block.
  // Out-of-range → 0.0 (safe-fail: never engages, §9).
  double quest_analog(int hand, int offset_within_block) const {
    const size_t idx =
        static_cast<size_t>(hand) * QUEST_RIGHT_OFFSET +
        static_cast<size_t>(std::max(offset_within_block, 0));
    if (idx >= quest_data_.size()) return 0.0;
    return quest_data_[idx];
  }

  // Keep-alive: thumb >= engage_min − hysteresis on BOTH hands (§4).
  bool quest_thumbs_held() {
    const double thr = quest_thumb_engage_min_ - quest_hold_hysteresis_;
    return quest_analog(0, quest_thumb_analog_offset_) >= thr &&
           quest_analog(1, quest_thumb_analog_offset_) >= thr;
  }

  // Early-release / go-fast: both triggers still held (same hysteresis) (§4).
  bool quest_triggers_held() {
    const double thr = quest_trigger_engage_min_ - quest_hold_hysteresis_;
    return quest_analog(0, quest_trigger_analog_offset_) >= thr &&
           quest_analog(1, quest_trigger_analog_offset_) >= thr;
  }

  // Re-read this instance's control pose from the latest /quest_state and apply
  // the optional frame alignment (§5/§7): Q' = R(quest_frame_rot_rpy)·Q,
  // translation scaled by quest_pos_scale. Called every quest tick (the target
  // is never latched at engage — §5).
  void update_quest_target() {
    const int block = (target_arm_ == "right") ? static_cast<int>(QUEST_RIGHT_OFFSET) : 0;
    if (quest_data_.size() < static_cast<size_t>(block) + 6)
      return;  // incomplete bridge data: hold the last target (safe-fail)
    const std::vector<float> &d = quest_data_;
    // Extrinsic XYZ RPY — matches quest_to_ros2.py torso_relative() (§3/§5)
    Eigen::Matrix3d R_quest =
        Eigen::AngleAxisd(d[block + 3], Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisd(d[block + 4], Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(d[block + 5], Eigen::Vector3d::UnitZ()).toRotationMatrix();
    quest_target_.linear() = quest_frame_rot_.linear() * R_quest;
    quest_target_.translation() =
        quest_pos_scale_ *
        (quest_frame_rot_.linear() *
         Eigen::Vector3d(d[block + 0], d[block + 1], d[block + 2]));
  }

  // §10 error metric: combined pos+rot vs the live quest target, measured on
  // the achieved pose, unlocked axes only. Returns a max-normalized scalar:
  // < 1.0 ⟺ err_pos < quest_approach_pos_m AND err_rot < quest_approach_ang_rad.
  double quest_error_norm() {
    if (!achieved_pose_valid_)
      return std::numeric_limits<double>::infinity();   // can't measure → never READY
    const double ep =
        (achieved_pose_map_.translation() - quest_target_.translation()).norm() /
        quest_approach_pos_m_;

    // Relative rotation achieved→target, decomposed as extrinsic XYZ RPY;
    // locked axes are irreducible (§10) so they are excluded from the error.
    Eigen::Matrix3d R_rel = achieved_pose_map_.linear().transpose() * quest_target_.linear();
    Eigen::Vector3d rpy = extract_rpy(R_rel);
    if (soft_lock_ee_roll_) rpy.x() = 0.0;
    if (soft_lock_ee_yaw_) rpy.z() = 0.0;
    Eigen::Matrix3d R_red =
        (Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix());
    const double cos_ang = std::clamp((R_red.trace() - 1.0) / 2.0, -1.0, 1.0);
    const double er = std::acos(cos_ang) / quest_approach_ang_rad_;

    return std::max(ep, er);
  }

  // §4 engage detection: runs every ARM tick while in SM_CONTROL. Arms only
  // from all-four-zero, then requires an uninterrupted THUMB+TRIGGER ≥ 0.5 on
  // both hands for quest_hold_engage_s. Any interruption disarms (§4).
  void detect_engage() {
    if (!quest_data_valid_)
      return;  // no /quest_state yet — nothing to engage on

    // Four engage floats read globally at fixed indices regardless of
    // target_arm_ — the gesture is 2-handed (§3/§4). "Thumb" = analog grip.
    const double lt = quest_analog(0, quest_trigger_analog_offset_);
    const double lth = quest_analog(0, quest_thumb_analog_offset_);
    const double rt = quest_analog(1, quest_trigger_analog_offset_);
    const double rth = quest_analog(1, quest_thumb_analog_offset_);

    const bool all_zero =
        lt < quest_release_min_ && lth < quest_release_min_ &&
        rt < quest_release_min_ && rth < quest_release_min_;
    const bool all_held =
        lt >= quest_trigger_engage_min_ && lth >= quest_thumb_engage_min_ &&
        rt >= quest_trigger_engage_min_ && rth >= quest_thumb_engage_min_;

    if (all_zero) {
      engage_armed_ = true;       // clean baseline: arm the count (§4)
      hold_started_ = false;
    } else if (engage_armed_ && all_held) {
      // Uninterrupted 4-button hold — run the 2 s countdown
      if (!hold_started_) {
        hold_started_ = true;
        hold_start_time_ = this->now();
      }
      if ((this->now() - hold_start_time_).seconds() >= quest_hold_engage_s_) {
        engage_from_hold();       // gated on a fresh achieved pose (§4)
      }
    } else {
      engage_armed_ = false;      // any interruption disarms; must re-zero (§4)
      hold_started_ = false;
    }
  }

  // The engage edge, gated on a live achieved pose (§4/§5). Re-captures
  // soft-lock centers so apply_ee_locks() does not fight the re-base (§6).
  void engage_from_hold() {
    if (!achieved_pose_valid_) {
      RCLCPP_WARN(this->get_logger(),
        "Quest engage ignored: no achieved pose yet (%s)", target_arm_.c_str());
      return;
    }
    const double age = (this->now() - achieved_pose_received_time_).seconds();
    if (age > quest_achieved_max_age_s_) {
      RCLCPP_WARN(this->get_logger(),
        "Quest engage ignored: achieved pose stale (%.2f s > %.2f s) (%s)",
        age, quest_achieved_max_age_s_, target_arm_.c_str());
      return;
    }

    if (soft_lock_ee_roll_ || soft_lock_ee_yaw_) {
      std::lock_guard<std::mutex> lock(ee_lock_mutex_);
      Eigen::Vector3d rpy = extract_rpy(achieved_pose_map_.linear());
      if (soft_lock_ee_roll_) soft_locked_roll_ = rpy.x();
      if (soft_lock_ee_yaw_) soft_locked_yaw_ = rpy.z();
      RCLCPP_INFO(this->get_logger(),
        "Re-captured soft-lock centers at engage (%s)", target_arm_.c_str());
    }

    ee_goal_ = achieved_pose_map_;   // no jump — the arm is already here (§5)
    enter_quest_state(QuestState::QUEST_APPROACH);
    RCLCPP_INFO(this->get_logger(),
      "Quest override engaged (%s) — slow approach toward hand", target_arm_.c_str());
  }

  // §4 per-tick evaluation in the QUEST states. Order: keep-alive, arrival
  // watchdog, then per-state transitions (READY when converged, early-release
  // lockout in APPROACH, go-fast on trigger release in READY).
  void step_quest_state_machine() {
    // §9 arrival watchdog: no /quest_state for quest_timeout_s → abort freeze.
    if ((this->now() - quest_last_msg_time_).seconds() > quest_timeout_s_) {
      abort_to_control("quest stream timeout");
      return;
    }

    // (1) keep-alive: either thumb below threshold → SM_CONTROL (§4)
    if (!quest_thumbs_held()) {
      abort_to_control("thumb released");
      return;
    }

    update_quest_target();   // live target re-read each tick (§5)

    switch (quest_state_) {
      case QuestState::QUEST_APPROACH: {
        // §10 no-progress stuck guard: no error progress for quest_no_progress_s
        const double err = quest_error_norm();
        if (err < quest_last_err_ - 1e-9) {
          quest_last_err_ = err;
          quest_last_progress_time_ = this->now();
        } else if ((this->now() - quest_last_progress_time_).seconds() >
                   quest_no_progress_s_) {
          publish_notification("quest_no_progress");
          abort_to_control("no progress toward target");
          return;
        }
        // (2) error < threshold → READY, notify (§4)
        if (err < 1.0) {
          enter_quest_state(QuestState::QUEST_READY);
          publish_notification("quest_approach_reached");
          RCLCPP_INFO(this->get_logger(),
            "Quest APPROACH complete (%s) — release TRIGGER to go fast",
            target_arm_.c_str());
          return;
        }
        // (3) either trigger released early (still not converged) → re-arm (§4)
        if (!quest_triggers_held()) {
          abort_to_control("trigger released before approach complete");
          return;
        }
        break;
      }
      case QuestState::QUEST_READY: {
        // (4) either trigger released (thumbs kept) → go fast (§4)
        if (!quest_triggers_held()) {
          enter_quest_state(QuestState::QUEST_TRACK);
          RCLCPP_INFO(this->get_logger(),
            "Quest TRACK — following hand 1:1 (%s)", target_arm_.c_str());
        }
        break;
      }
      case QuestState::QUEST_TRACK:
        break;  // keep-alive / watchdog only
      default:
        break;
    }
  }

  // §6 interpolation: linear blend translation, quaternion slerp. READY holds
  // (w = 0) until the trigger release confirms TRACK (§4).
  void step_quest_interpolation() {
    double w = 0.0;
    if (quest_state_ == QuestState::QUEST_APPROACH) w = quest_w_slow_;
    else if (quest_state_ == QuestState::QUEST_TRACK) w = quest_w_fast_;

    ee_goal_.translation() =
        (1.0 - w) * ee_goal_.translation() + w * quest_target_.translation();

    Eigen::Quaterniond q_cur(ee_goal_.linear());
    Eigen::Quaterniond q_tgt(quest_target_.linear());
    Eigen::Quaterniond q_new = q_cur.slerp(w, q_tgt);
    ee_goal_.linear() = q_new.toRotationMatrix();
  }

  // Every path back to SM_CONTROL does exactly this (§4 "Abort"): stop
  // listening to the quest, freeze ee_goal_ where it is, disarm the count.
  void abort_to_control(const char *reason) {
    enter_quest_state(QuestState::SM_CONTROL);
    engage_armed_ = false;
    hold_started_ = false;
    RCLCPP_INFO(this->get_logger(),
      "Quest override off (%s) — goal frozen, count disarmed (%s)",
      reason, target_arm_.c_str());
  }

  // Set the new state, publish /quest/<arm>/active on the SM_CONTROL crossing,
  // and seed the no-progress baseline on APPROACH entry.
  void enter_quest_state(QuestState s) {
    if (quest_state_ == s) return;
    const bool was_active = (quest_state_ != QuestState::SM_CONTROL);
    const bool now_active = (s != QuestState::SM_CONTROL);
    quest_state_ = s;
    if (was_active != now_active) {
      std_msgs::msg::Bool flag;
      flag.data = now_active;
      quest_active_pub_->publish(flag);
    }
    if (s == QuestState::QUEST_APPROACH) {
      update_quest_target();
      quest_last_err_ = quest_error_norm();
      quest_last_progress_time_ = this->now();
    }
  }

  // One joy_hand instance only may fire notifications (the right-hand instance
  // is launched with quest_publish_notifications=false, §7).
  void publish_notification(const std::string &payload) {
    if (!quest_publish_notifications_) return;
    std_msgs::msg::String msg;
    msg.data = payload;
    quest_notification_pub_->publish(msg);
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

    // ── Quest override (quest_teleop_plan §4-§7, Task 2) ──
    // Runs every ARM tick, active or not. Its input is /quest_state, not /joy,
    // so it runs before the joy_received_ gate below. The single-threaded
    // executor serializes the timer and all callbacks → no new mutex needed
    // (same pattern as precision_mode_).
    if (quest_state_ == QuestState::SM_CONTROL) {
      detect_engage();   // all-4-zero → 2 s hold → engage (§4)
    }
    if (quest_state_ != QuestState::SM_CONTROL) {
      step_quest_state_machine();   // keep-alive / go-fast / watchdog (§4/§9)
      if (quest_state_ == QuestState::SM_CONTROL) {
        // Aborted: goal frozen where it is, count disarmed (§4 "Abort").
        publish_pose();
        return;
      }
      step_quest_interpolation();   // slerp ee_goal_ toward quest target (§6)
      {
        std::lock_guard<std::mutex> lock(ee_lock_mutex_);
        apply_ee_locks();   // soft-locks still enforced in quest mode (§6)
      }
      publish_pose();
      return;   // SpaceMouse velocity suppressed while quest is active (§7)
    }

    sensor_msgs::msg::Joy joy;
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      if (!joy_received_) return;
      joy = latest_joy_;
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
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr quest_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr quest_active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quest_notification_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string target_arm_;
  std::mutex joy_mutex_;
  sensor_msgs::msg::Joy latest_joy_;
  bool joy_received_ {false};
  std::string current_mode_ {"BASE"};

  // ── World-frame re-base state (quest_teleop_plan §11, Task 1) ──
  Eigen::Isometry3d achieved_pose_map_ {Eigen::Isometry3d::Identity()};
  bool achieved_pose_valid_ {false};
  rclcpp::Time achieved_pose_received_time_;

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

  // ── Quest override state (quest_teleop_plan §4-§10, Task 2) ──
  QuestState quest_state_ {QuestState::SM_CONTROL};   // quest_active_ ⇔ != SM_CONTROL
  bool engage_armed_ {false};      // §4: count arms only from all-four-zero
  bool hold_started_ {false};
  rclcpp::Time hold_start_time_;
  rclcpp::Time quest_last_msg_time_;      // §9 arrival watchdog
  double quest_last_err_ {0.0};           // §10 no-progress tracking
  rclcpp::Time quest_last_progress_time_;
  std::vector<float> quest_data_;         // cached /quest_state flat array
  bool quest_data_valid_ {false};
  Eigen::Isometry3d quest_target_ {Eigen::Isometry3d::Identity()};  // live interpolation target

  // Cached quest parameters
  std::string quest_state_topic_ {"/quest_state"};
  std::string quest_active_topic_;
  std::string quest_notification_topic_ {"/ffw_control/notification"};
  bool quest_publish_notifications_ {true};
  double quest_hold_engage_s_ {2.0};
  double quest_release_min_ {0.1};
  int quest_thumb_analog_offset_ {7};
  int quest_trigger_analog_offset_ {6};
  double quest_thumb_engage_min_ {0.5};
  double quest_trigger_engage_min_ {0.5};
  double quest_w_slow_ {0.02};
  double quest_w_fast_ {0.15};
  double quest_approach_pos_m_ {0.05};
  double quest_approach_ang_rad_ {0.10};
  double quest_timeout_s_ {0.5};
  double quest_achieved_max_age_s_ {0.5};
  double quest_hold_hysteresis_ {0.05};
  double quest_no_progress_s_ {5.0};
  Eigen::Isometry3d quest_frame_rot_ {Eigen::Isometry3d::Identity()};  // optional alignment
  double quest_pos_scale_ {1.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyHand>());
  rclcpp::shutdown();
  return 0;
}
