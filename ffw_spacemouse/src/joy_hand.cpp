#include <algorithm>
#include <array>
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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ffw_spacemouse_msgs/msg/left_control_override.hpp"

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
    // Left-quest-trigger precision override (right arm only): continuous
    // scale from 1.0 (trigger released) down to this floor (trigger fully held).
    this->declare_parameter("quest_precision_min_scale", 0.2);

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

    // Remote control override: a single software bool that plays the role of the
    // Quest engage gesture. When TRUE the spacemouse stream is suppressed at the
    // source and the external /quest goal stream (ZMQ gateway) is authoritative.
    this->declare_parameter("control_override_topic", "/control_override");

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
    this->declare_parameter("quest_w_slow", 0.15);
    this->declare_parameter("quest_approach_pos_m", 0.05);
    this->declare_parameter("quest_approach_ang_rad", 0.10);
    this->declare_parameter("quest_timeout_s", 0.5);        // no /quest_state → auto-return
    this->declare_parameter("quest_achieved_max_age_s", 0.5); // engage re-base: achieved age limit
    this->declare_parameter("quest_hold_hysteresis", 0.05); // keep-alive/early-release band (§4)
    this->declare_parameter("quest_frame_rot_rpy", std::vector<double>{0.0, 0.0, 0.0}); // optional alignment
    this->declare_parameter("quest_pos_scale", 1.0);
    this->declare_parameter("quest_xy_scale", 3.0);     // TRACK reach gain: stacks with the velocity lead (§6 TEST 3)
    this->declare_parameter("quest_lead_t", 0.25);      // §6 TEST 3: feedforward lead distance = hand velocity × T_lead
    this->declare_parameter("quest_vel_alpha", 0.2);    // §6 TEST 3: EMA factor on the smoothed hand velocity
    this->declare_parameter("quest_debug_engage", false);  // log detect_engage edges (§4)
    this->declare_parameter("quest_debug_error", false);   // §10: print APPROACH error budget

    quest_state_topic_ = this->get_parameter("quest_state_topic").as_string();
    quest_active_topic_ = this->get_parameter("quest_active_topic_prefix").as_string() +
                          "/" + target_arm_ + "/active";
    // Per-arm state topic (quest_teleop_plan §8b): published once per state
    // change so the solver can pick a per-state leash — 1.5cm during APPROACH
    // (before trigger release), 6cm after (TRACK).
    quest_state_pub_topic_ = this->get_parameter("quest_active_topic_prefix").as_string() +
                             "/" + target_arm_ + "/state";
    // Per-arm trigger feed: live trigger analog each quest tick. In TRACK the
    // solver maps it to the gripper (1 → close, 0 → open); CTRL/APPROACH ignore
    // it so the 4-way engage hold never slams the gripper (quest_teleop_plan §11).
    quest_trigger_pub_topic_ = this->get_parameter("quest_active_topic_prefix").as_string() +
                               "/" + target_arm_ + "/trigger";
    quest_notification_topic_ = this->get_parameter("quest_notification_topic").as_string();
    quest_publish_notifications_ = this->get_parameter("quest_publish_notifications").as_bool();
    quest_hold_engage_s_ = this->get_parameter("quest_hold_engage_s").as_double();
    quest_release_min_ = this->get_parameter("quest_release_min").as_double();
    quest_thumb_analog_offset_ = this->get_parameter("quest_thumb_analog_offset").as_int();
    quest_trigger_analog_offset_ = this->get_parameter("quest_trigger_analog_offset").as_int();
    quest_thumb_engage_min_ = this->get_parameter("quest_thumb_engage_min").as_double();
    quest_trigger_engage_min_ = this->get_parameter("quest_trigger_engage_min").as_double();
    quest_w_slow_ = this->get_parameter("quest_w_slow").as_double();
    quest_approach_pos_m_ = this->get_parameter("quest_approach_pos_m").as_double();
    quest_approach_ang_rad_ = this->get_parameter("quest_approach_ang_rad").as_double();
    quest_timeout_s_ = this->get_parameter("quest_timeout_s").as_double();
    quest_achieved_max_age_s_ = this->get_parameter("quest_achieved_max_age_s").as_double();
    quest_hold_hysteresis_ = this->get_parameter("quest_hold_hysteresis").as_double();
    quest_pos_scale_ = this->get_parameter("quest_pos_scale").as_double();
    quest_xy_scale_ = this->get_parameter("quest_xy_scale").as_double();
    quest_lead_t_ = this->get_parameter("quest_lead_t").as_double();
    quest_vel_alpha_ = this->get_parameter("quest_vel_alpha").as_double();
    quest_debug_engage_ = this->get_parameter("quest_debug_engage").as_bool();
    quest_debug_error_ = this->get_parameter("quest_debug_error").as_bool();

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
    // Quest mode publishes the same ee_goal_ to its own topic: the solver treats
    // /spacemouse goals as deltas and /quest goals as absolute poses (§10 fix).
    std::string quest_topic_name = "/quest/" + target_arm_ + "/ee_target_pose";
    quest_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(quest_topic_name, 10);

    // Delta stream (deltas out): what the human commanded on the /spacemouse
    // delta path this tick, map frame, AFTER the lock/limit-profile clamps.
    // linear = dx,dy,dz (m); angular = drx,dry,drz (rad, extrinsic-XYZ RPY).
    // The solver recovers the same quantity by differencing consecutive
    // /spacemouse/<arm>/ee_target_pose goals; this publishes it explicitly
    // for supervisors / demo loggers (relayed to the ZMQ side as EEDelta).
    std::string delta_topic_name = "/spacemouse/" + target_arm_ + "/ee_target_delta";
    delta_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(delta_topic_name, 10);

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

    // Left Meta Quest controller override (quest_to_ros2.py): the left
    // trigger continuously scales this (right-arm) SpaceMouse's sensitivity.
    // Only the right-arm instance reacts — left_ctrl_trigger_ stays 0.0
    // (no-op scale of 1.0) for the left-arm instance, same no-mutex pattern
    // as precision_mode_ (single executor thread).
    if (target_arm_ == "right") {
      left_ctrl_sub_ = this->create_subscription<ffw_spacemouse_msgs::msg::LeftControlOverride>(
        "/quest/left/control_override", 10,
        [this](const ffw_spacemouse_msgs::msg::LeftControlOverride::SharedPtr msg) {
            left_ctrl_trigger_ = msg->trigger;
            left_ctrl_grip_ = msg->grip;
        });
    }

    // ── Remote control override (single bool gate, mirrors the Quest engage) ──
    // One bool on /control_override replaces the 2 s trigger gesture: when TRUE,
    // joy_hand stops publishing /spacemouse/<arm>/ee_target_pose entirely, so the
    // gateway's /quest/<arm>/ee_target_pose goals become the only goal stream and
    // win in the solver (no delta re-assert). No mutex: single executor thread,
    // same pattern as precision_mode_.
    control_override_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      this->get_parameter("control_override_topic").as_string(), 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
          bool now = msg->data;
          // Override release edge (TRUE→FALSE): while the override held, the
          // external /quest goal moved the arm out from under the frozen ee_goal_,
          // so a plain resume would re-assert the pre-override pose. Adopt the
          // real pose now — same re-base as the ARM switch (§11), so spacemouse
          // deltas continue from where the arm actually is.
          if (control_override_ && !now) {
            if (current_mode_ == "ARM" && achieved_pose_valid_) {
              ee_goal_ = achieved_pose_map_;
              RCLCPP_INFO(this->get_logger(),
                "Rebased ee_goal_ to achieved pose on override release (%s)",
                target_arm_.c_str());
            }
            // The override branch below forces the solver into TRACK every tick
            // so the gateway's gripper trigger is obeyed. Restore the real quest
            // state on release — enter_quest_state only publishes on transitions,
            // so this needs an explicit publish or the solver stays in TRACK.
            if (quest_state_pub_) {
              std_msgs::msg::String state_msg;
              state_msg.data = quest_state_name();
              quest_state_pub_->publish(state_msg);
            }
          }
          control_override_ = now;
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
          // After a machine-state resync (joint-state recovery) only a pose
          // published AFTER the resync may anchor us — the solver re-snaps
          // MuJoCo to the real joints and then publishes the fresh pose, so
          // any message stamped before our resync is still the stale pre-sync
          // simulation pose. Skip it and keep waiting for the fresh one.
          if (rclcpp::Time(msg->header.stamp).nanoseconds() <
              resync_stamp_.nanoseconds())
            return;
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

    // ── Machine-state resync subscription ─────────────────────────
    // joy_base fires this on joint-state recovery: drop the achieved-pose
    // anchor so the next (post-sync) achieved pose re-bases ee_goal_ onto the
    // real robot. Only meaningful in ARM — in BASE the solver keeps streaming
    // fresh achieved poses anyway, and the ARM-switch path re-bases on entry.
    resync_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/teleop_resync", 10, [this](const std_msgs::msg::Empty::SharedPtr) {
          if (current_mode_ == "ARM") {
            achieved_pose_valid_ = false;
            resync_stamp_ = this->now();
            RCLCPP_INFO(this->get_logger(),
              "Resync requested — waiting for fresh achieved pose (%s)",
              target_arm_.c_str());
          }
      });

    // ── Pose-change subscription (solver → mapper) ─────────────────
    // Fired by ffw_ik_solver_teleop when a pose load / home reset homing
    // completes: the arm settled at a pose the mapper never commanded, so
    // re-base ee_goal_ onto the achieved pose immediately. Without this the
    // goal stays anchored at the pre-load pose and the limit profile (a
    // world-frame clamp on the goal) applies relative to the stale reference —
    // the effective range shifts by the pose displacement.
    pose_changed_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/teleop/pose_changed", 10, [this](const std_msgs::msg::Empty::SharedPtr) {
          if (current_mode_ == "ARM" && achieved_pose_valid_) {
            ee_goal_ = achieved_pose_map_;
            RCLCPP_INFO(this->get_logger(),
              "Pose changed — ee_goal_ re-based to achieved pose (%s)",
              target_arm_.c_str());
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
        } else if (axis_arm.find("pitch") != std::string::npos) {
          lock_flag = &soft_lock_ee_pitch_;
          locked_value = &soft_locked_pitch_;
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
          double sin_pitch = R(0, 2);
          double pitch, yaw;
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
          if (lock_flag == &soft_lock_ee_pitch_) *locked_value = pitch;
          const char *axis_name = (lock_flag == &soft_lock_ee_roll_) ? "roll"
                               : (lock_flag == &soft_lock_ee_yaw_) ? "yaw"
                               : "pitch";
          RCLCPP_INFO(this->get_logger(), "EE %s locked at %.3f rad for %s",
            axis_name, *locked_value, target_arm_.c_str());
        }
      });

    // ── Manual limit profile subscription (CLI-created) ──
    // Format: "set <arm> px_min px_max py_min py_max pz_min pz_max roll_min
    // roll_max pitch_min pitch_max yaw_min yaw_max" or "clear <arm>". Bounds in
    // meters/radians, absolute world frame ("map"); survives resyncs. The arm
    // token matches by suffix like /ik_solver/ee_lock above.
    limit_profile_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/teleop/limit_profile", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(ee_lock_mutex_);
        std::stringstream ss(msg->data);
        std::string action, arm;
        ss >> action >> arm;
        // Accept the bare "l"/"r" token (CLI limit-profile messages) as well as
        // the "_l"/"_r" suffix form (ee_lock style), so both senders match.
        bool matches_my_arm = false;
        if (target_arm_ == "left" && (arm == "l" || arm.find("_l") != std::string::npos))
          matches_my_arm = true;
        else if (target_arm_ == "right" && (arm == "r" || arm.find("_r") != std::string::npos))
          matches_my_arm = true;
        if (!matches_my_arm) return;

        if (action == "clear") {
          limit_profile_.active = false;
          RCLCPP_INFO(this->get_logger(), "Limit profile cleared (%s)", target_arm_.c_str());
          return;
        }
        if (action != "set") {
          RCLCPP_WARN(this->get_logger(), "Invalid limit_profile action: '%s'", action.c_str());
          return;
        }

        double bounds[12];
        for (double &b : bounds) {
          if (!(ss >> b)) {
            RCLCPP_WARN(this->get_logger(),
              "Invalid limit_profile set format — need 12 bounds, got partial parse");
            return;
          }
        }
        LimitProfile &p = limit_profile_;
        p.px_min = bounds[0]; p.px_max = bounds[1];
        p.py_min = bounds[2]; p.py_max = bounds[3];
        p.pz_min = bounds[4]; p.pz_max = bounds[5];
        p.roll_min = bounds[6]; p.roll_max = bounds[7];
        p.pitch_min = bounds[8]; p.pitch_max = bounds[9];
        p.yaw_min = bounds[10]; p.yaw_max = bounds[11];
        p.active = true;
        RCLCPP_INFO(this->get_logger(),
          "Limit profile set (%s): x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] "
          "roll[%.3f,%.3f] pitch[%.3f,%.3f] yaw[%.3f,%.3f]",
          target_arm_.c_str(), p.px_min, p.px_max, p.py_min, p.py_max, p.pz_min, p.pz_max,
          p.roll_min, p.roll_max, p.pitch_min, p.pitch_max, p.yaw_min, p.yaw_max);
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
    quest_state_pub_ =
      this->create_publisher<std_msgs::msg::String>(quest_state_pub_topic_, 10);
    quest_trigger_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(quest_trigger_pub_topic_, 10);

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
  enum class QuestState { SM_CONTROL, QUEST_APPROACH, QUEST_TRACK };

  // ── EE lock helpers ─────────────────────────────────────────────

  static Eigen::Vector3d extract_rpy(const Eigen::Matrix3d &R) {
    // Extrinsic XYZ Euler: R = Rx(roll) * Ry(pitch) * Rz(yaw)
    double pitch = std::asin(std::clamp(R(0, 2), -1.0, 1.0));
    double yaw = std::atan2(-R(0, 1), R(0, 0));
    double roll = std::atan2(-R(1, 2), R(2, 2));
    return Eigen::Vector3d(roll, pitch, yaw);
  }

  static Eigen::Matrix3d rpy_to_matrix(double roll, double pitch, double yaw) {
    // Extrinsic XYZ Euler: R = Rx(roll) * Ry(pitch) * Rz(yaw)
    return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix() *
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }

  // ── Manual limit profile (CLI-created per-arm position+orientation box) ──
  // Bounds in meters and radians, clamped onto the absolute world-frame goal
  // every tick before publishing. Guarded by ee_lock_mutex_ (subscription
  // callback writes; apply_limit_profile() reads under the caller's lock).
  struct LimitProfile {
    bool active {false};
    double px_min {0}, px_max {0};
    double py_min {0}, py_max {0};
    double pz_min {0}, pz_max {0};
    double roll_min {0}, roll_max {0};
    double pitch_min {0}, pitch_max {0};
    double yaw_min {0}, yaw_max {0};
  };

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
    // Clamp locked roll/yaw/pitch axes within slack of their locked centers
    if (!soft_lock_ee_roll_ && !soft_lock_ee_yaw_ && !soft_lock_ee_pitch_)
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
    if (soft_lock_ee_pitch_)
      pitch = std::clamp(pitch, soft_locked_pitch_ - ee_orientation_slack_,
                               soft_locked_pitch_ + ee_orientation_slack_);

    ee_goal_.linear() = rpy_to_matrix(roll, pitch, yaw);
  }

  void apply_limit_profile() {
    // Clamp the absolute world-frame goal inside the manual profile box. The
    // profile is the outer hard envelope: call AFTER apply_ee_locks() so a
    // lock center outside the box can never pull the goal past a bound.
    // Caller holds ee_lock_mutex_ (same contract as apply_ee_locks()).
    if (!limit_profile_.active) return;

    const LimitProfile &p = limit_profile_;
    Eigen::Vector3d t = ee_goal_.translation();
    t.x() = std::clamp(t.x(), p.px_min, p.px_max);
    t.y() = std::clamp(t.y(), p.py_min, p.py_max);
    t.z() = std::clamp(t.z(), p.pz_min, p.pz_max);
    ee_goal_.translation() = t;

    Eigen::Vector3d rpy = extract_rpy(ee_goal_.linear());
    double roll = std::clamp(rpy.x(), p.roll_min, p.roll_max);
    double pitch = std::clamp(rpy.y(), p.pitch_min, p.pitch_max);
    double yaw = std::clamp(rpy.z(), p.yaw_min, p.yaw_max);
    ee_goal_.linear() = rpy_to_matrix(roll, pitch, yaw);
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

  // §10 error budget: combined pos+rot vs the live quest target, measured on
  // the achieved pose, unlocked axes only. err < 1.0 ⟺ err_pos <
  // quest_approach_pos_m AND err_rot < quest_approach_ang_rad. The ep/er
  // breakdown feeds the §10 debug so the slow→fast convergence is inspectable.
  struct QuestErrorBudget {
    double ep {std::numeric_limits<double>::infinity()};
    double er {std::numeric_limits<double>::infinity()};
    double err {std::numeric_limits<double>::infinity()};
  };
  QuestErrorBudget quest_error_budget() {
    QuestErrorBudget out;
    if (!achieved_pose_valid_)
      return out;   // can't measure → never READY
    out.ep =
        (achieved_pose_map_.translation() - quest_target_.translation()).norm() /
        quest_approach_pos_m_;

    // Relative rotation achieved→target, decomposed as extrinsic XYZ RPY;
    // locked axes are irreducible (§10) so they are excluded from the error.
    Eigen::Matrix3d R_rel = achieved_pose_map_.linear().transpose() * quest_target_.linear();
    Eigen::Vector3d rpy = extract_rpy(R_rel);
    if (soft_lock_ee_roll_) rpy.x() = 0.0;
    if (soft_lock_ee_pitch_) rpy.y() = 0.0;
    if (soft_lock_ee_yaw_) rpy.z() = 0.0;
    Eigen::Matrix3d R_red =
        (Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix());
    const double cos_ang = std::clamp((R_red.trace() - 1.0) / 2.0, -1.0, 1.0);
    out.er = std::acos(cos_ang) / quest_approach_ang_rad_;

    out.err = std::max(out.ep, out.er);
    return out;
  }

  const char *quest_state_name() const {
    switch (quest_state_) {
      case QuestState::SM_CONTROL: return "CTRL";
      case QuestState::QUEST_APPROACH: return "APPROACH";
      case QuestState::QUEST_TRACK: return "TRACK";
    }
    return "?";
  }

  // §10 debug (quest_debug_error=true): print the error budget on a throttle
  // while quest is active, plus the three position distances that explain it —
  // achieved→target (what READY measures), goal→target (interpolation
  // residual), achieved→goal (arm tracking lag). Slow→fast failure shows up
  // immediately: err stuck ≥1 while one of these is large and not shrinking.
  void debug_error() {
    if (!quest_debug_error_) return;
    if (this->now().seconds() - last_debug_error_s_ < 0.2) return;
    last_debug_error_s_ = this->now().seconds();
    const auto b = quest_error_budget();
    const double at =
        (achieved_pose_map_.translation() - quest_target_.translation()).norm();
    const double gt =
        (ee_goal_.translation() - quest_target_.translation()).norm();
    const double ag =
        (achieved_pose_map_.translation() - ee_goal_.translation()).norm();
    RCLCPP_INFO(this->get_logger(),
      "quest %s [%s]: err=%.2f ep=%.2f er=%.2f  |ach->tgt|=%.3fm |goal->tgt|=%.3fm |ach->goal|=%.3fm",
      target_arm_.c_str(), quest_state_name(), b.err, b.ep, b.er, at, gt, ag);
  }

  // §4 debug (quest_debug_engage=true): log each detect_engage edge — arming,
  // hold start, disarm — once per transition with the four raw analog values,
  // so the receive path is visible without flooding the 100 Hz control tick.
  void debug_engage(const std::string &label, double lt, double lth,
                    double rt, double rth) {
    if (!quest_debug_engage_)
      return;
    if (label == last_debug_engage_label_)
      return;  // transition-only
    last_debug_engage_label_ = label;
    RCLCPP_INFO(this->get_logger(),
      "quest engage %s: %s  lt=%.2f lth=%.2f rt=%.2f rth=%.2f",
      target_arm_.c_str(), label.c_str(), lt, lth, rt, rth);
  }

  // §4 engage detection: runs every ARM tick while in SM_CONTROL. Arms only
  // from all-four-zero, then requires an uninterrupted THUMB+TRIGGER ≥ 0.5 on
  // both hands for quest_hold_engage_s. Any interruption disarms (§4).
  void detect_engage() {
    if (!quest_data_valid_) {
      debug_engage("NO_DATA", 0.0, 0.0, 0.0, 0.0);
      return;  // no /quest_state yet — nothing to engage on
    }
    // Right-hand floats live at indices 22-23 (rt/rth) — a short payload
    // would read 0.0 via quest_analog and fake an all-zero baseline (§3).
    if (quest_data_.size() < 24) {
      debug_engage("MALFORMED(size=" + std::to_string(quest_data_.size()) + ")",
                   0.0, 0.0, 0.0, 0.0);
      return;
    }

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
      last_debug_progress_s_ = -1.0;
      debug_engage("COUNT-READY (all-zero baseline)", lt, lth, rt, rth);
    } else if (all_held) {
      if (!engage_armed_) {
        // All four pressed but no all-zero baseline was seen first — the
        // release-then-hold sequence (§4) has not been armed, so this squeeze
        // does not count. Tell the user instead of silently ignoring it.
        debug_engage("SQUEEZED (release to all-zero first)", lt, lth, rt, rth);
      } else {
        // Uninterrupted 4-button hold — run the 2 s countdown
        if (!hold_started_) {
          hold_started_ = true;
          hold_start_time_ = this->now();
          last_debug_progress_s_ = 0.0;
          debug_engage("HOLDING (count started)", lt, lth, rt, rth);
        }
        // Show the countdown tick so the 2 s requirement is visible
        if (quest_debug_engage_) {
          const double el = (this->now() - hold_start_time_).seconds();
          if (el - last_debug_progress_s_ >= 0.5) {
            last_debug_progress_s_ = el;
            RCLCPP_INFO(this->get_logger(),
              "quest engage %s: HOLDING %.1f/%.1fs  lt=%.2f lth=%.2f rt=%.2f rth=%.2f",
              target_arm_.c_str(), el, quest_hold_engage_s_, lt, lth, rt, rth);
          }
        }
        if ((this->now() - hold_start_time_).seconds() >= quest_hold_engage_s_) {
          engage_from_hold();     // gated on a fresh achieved pose (§4)
        }
      }
    } else {
      // Partial: some fingers down, not yet all four. This is the normal
      // start-of-squeeze — the count has NOT begun, so do not cancel yet.
      // Only a break after the hold actually started disarms (§4).
      if (hold_started_) {
        engage_armed_ = false;    // hold broke mid-count; must re-zero (§4)
        hold_started_ = false;
        last_debug_progress_s_ = -1.0;
        debug_engage("CANCELLED (hold broke — need all 4 ≥0.5)", lt, lth, rt, rth);
      } else {
        // Armed but mid-squeeze: wait for all four ≥0.5. Stream live values
        // so a held gesture is visible even before the count starts (§4).
        if (quest_debug_engage_ &&
            this->now().seconds() - last_debug_live_s_ > 0.5) {
          last_debug_live_s_ = this->now().seconds();
          RCLCPP_INFO(this->get_logger(),
            "quest engage %s: LIVE  lt=%.2f lth=%.2f rt=%.2f rth=%.2f",
            target_arm_.c_str(), lt, lth, rt, rth);
        }
      }
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

    if (soft_lock_ee_roll_ || soft_lock_ee_yaw_ || soft_lock_ee_pitch_) {
      std::lock_guard<std::mutex> lock(ee_lock_mutex_);
      Eigen::Vector3d rpy = extract_rpy(achieved_pose_map_.linear());
      if (soft_lock_ee_roll_) soft_locked_roll_ = rpy.x();
      if (soft_lock_ee_yaw_) soft_locked_yaw_ = rpy.z();
      if (soft_lock_ee_pitch_) soft_locked_pitch_ = rpy.y();
      RCLCPP_INFO(this->get_logger(),
        "Re-captured soft-lock centers at engage (%s)", target_arm_.c_str());
    }

    ee_goal_ = achieved_pose_map_;   // no jump — the arm is already here (§5)
    enter_quest_state(QuestState::QUEST_APPROACH);
    RCLCPP_INFO(this->get_logger(),
      "Quest override engaged (%s) — slow approach toward hand", target_arm_.c_str());
  }

  // §4 per-tick evaluation in the QUEST states. Order: keep-alive, arrival
  // watchdog, then per-state transitions (go-fast on trigger release from
  // APPROACH; no convergence/error requirement — trigger release always wins).
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

    debug_error();   // §10: print APPROACH error budget when enabled

    switch (quest_state_) {
      case QuestState::QUEST_APPROACH: {
        // No convergence/error gate: releasing the triggers (thumbs kept)
        // always enters fast mode, whatever the approach error is (§4).
        if (!quest_triggers_held()) {
          enter_quest_state(QuestState::QUEST_TRACK);
          RCLCPP_INFO(this->get_logger(),
            "Quest TRACK — raw pose published directly (§6 TEST, %s)",
            target_arm_.c_str());
        }
        break;
      }
      case QuestState::QUEST_TRACK:
        break;  // keep-alive / watchdog only
      default:
        break;
    }
  }

  // §6 interpolation (TEST 3): BOTH states share the same led goal — the raw
  // hand pose advanced by v·T_lead (x,y reach-scaled). TRACK publishes it
  // direct; APPROACH glides toward it at a low interpolation rate (w_slow) so
  // the approach stays slow and careful, and the arm is already riding the led
  // trajectory when the triggers release — no route change, no catch-up burst
  // at the APPROACH→TRACK crossing. Rotation slerps at w_slow in APPROACH (so
  // grabbing doesn't snap hand orientation onto the arm) and goes raw in TRACK.
  void step_quest_interpolation() {
    // §6 TEST 3: velocity feedforward — goal = raw hand pose + v·T_lead. The
    // lead is nonzero only while the hand is moving (v≈0 at rest → no parked
    // offset, unlike the reach scale alone), and it pushes the goal v·T_lead
    // ahead of the hand so the solver's 3cm leash/damper never holds the arm
    // back. It stacks on top of quest_xy_scale (2.0 default): scale amplifies
    // the absolute reach, lead adds motion-dependent speed.
    const Eigen::Vector3d hand_pos = quest_target_.translation();
    update_quest_velocity(hand_pos);   // keep the feedforward velocity warm
    Eigen::Vector3d led_target = hand_pos;
    if (quest_vel_seeded_) led_target += quest_vel_f_ * quest_lead_t_;
    led_target.x() *= quest_xy_scale_;
    led_target.y() *= quest_xy_scale_;

    if (quest_state_ == QuestState::QUEST_TRACK) {
      ee_goal_.translation() = led_target;
      ee_goal_.linear() = quest_target_.linear();   // rotation goes raw in TRACK
      return;
    }

    // APPROACH: slow glide toward the led target. Translation and rotation both
    // move w_slow (~15%/tick) of the way each tick, so the arm eases in and
    // decelerates as the gap closes — a careful approach, not a snap.
    const Eigen::Vector3d cur_pos = ee_goal_.translation();
    ee_goal_.translation() = cur_pos + quest_w_slow_ * (led_target - cur_pos);

    Eigen::Quaterniond q_cur(ee_goal_.linear());
    Eigen::Quaterniond q_tgt(quest_target_.linear());
    Eigen::Quaterniond q_new = q_cur.slerp(quest_w_slow_, q_tgt);
    ee_goal_.linear() = q_new.toRotationMatrix();
  }

  // §6 TEST 3: windowed-secant hand velocity + EMA smoothing, one call per
  // tick. Pushes the raw hand pose into a history ring, then estimates the
  // velocity over the oldest→newest span (the linear least-squares slope for
  // constant-velocity motion, so per-sample jitter averages out — noise ÷N vs
  // the single-tick Δ) and low-passes it so the commanded lead doesn't jitter.
  // Runs in APPROACH too so the filter is warm the instant triggers release.
  void update_quest_velocity(const Eigen::Vector3d &p) {
    const rclcpp::Time now = this->now();
    quest_hist_p_[quest_hist_head_] = p;
    quest_hist_t_[quest_hist_head_] = now;
    quest_hist_head_ = (quest_hist_head_ + 1) % kQuestHistN;
    quest_hist_count_ = std::min(quest_hist_count_ + 1, kQuestHistN);
    if (quest_hist_count_ < 2) return;   // need ≥ 2 samples for a secant
    const size_t oldest = (quest_hist_head_ + kQuestHistN - quest_hist_count_) % kQuestHistN;
    const double dt = (now - quest_hist_t_[oldest]).seconds();
    if (dt < 1e-4) return;               // zero elapsed — skip this tick
    const Eigen::Vector3d v = (p - quest_hist_p_[oldest]) / dt;
    if (!quest_vel_seeded_) {
      quest_vel_f_ = v;                  // first valid window: jump-start the EMA
      quest_vel_seeded_ = true;
    } else {
      quest_vel_f_ += quest_vel_alpha_ * (v - quest_vel_f_);
    }
  }

  // Every path back to SM_CONTROL does exactly this (§4 "Abort"): stop
  // listening to the quest, re-base ee_goal_ onto the achieved pose, disarm
  // the count.
  //
  // Re-base (not just freeze) is required because step_quest_interpolation's
  // velocity feedforward (§6 TEST 3) intentionally runs ee_goal_ AHEAD of the
  // hand/arm by v*T_lead while engaged, so "where it is" at abort time can sit
  // measurably past where the arm actually is — worse the faster the operator
  // was moving. The mapper's delta topic goes silent for the whole quest
  // session (joy_hand publishes on the absolute quest_pose_pub_ topic instead),
  // so its baseline (last_mapper_*) is still parked at the pre-engagement pose.
  // The first post-abort SpaceMouse tick then hands the solver a goal far from
  // that baseline; the solver's own re-base guard (kMapperRebaseDist) adopts it
  // directly on the assumption "the arm is already there" (true for every other
  // re-base site — ARM switch, override release, quest engage — because they
  // all anchor on achieved_pose_map_ explicitly). Skipping this re-base breaks
  // that assumption and the solver adopts the lead-ahead goal as-is: a real
  // jump toward it, walked back over the next few ticks, before SpaceMouse
  // control feels normal again.
  void abort_to_control(const char *reason) {
    enter_quest_state(QuestState::SM_CONTROL);
    engage_armed_ = false;
    hold_started_ = false;
    if (achieved_pose_valid_) {
      ee_goal_ = achieved_pose_map_;
    }
    RCLCPP_INFO(this->get_logger(),
      "Quest override off (%s) — goal rebased to achieved, count disarmed (%s)",
      reason, target_arm_.c_str());
  }

  // Set the new state, publish /quest/<arm>/active on the SM_CONTROL crossing,
  // and reset the velocity window on APPROACH entry (TEST 3).
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
    // §8b: publish the state on every transition so the solver's leash can
    // follow (APPROACH → 1.5cm, TRACK → 3cm). Guarded: enter_quest_state can
    // run from the mode callback, which is only reachable after construction.
    if (quest_state_pub_) {
      std_msgs::msg::String state_msg;
      state_msg.data = quest_state_name();
      quest_state_pub_->publish(state_msg);
    }
    if (s == QuestState::QUEST_APPROACH) {
      update_quest_target();
      // TEST 3: nothing to seed — APPROACH glides from the current ee_goal_
      // toward the led target, so engaging never jumps the arm. Reset the
      // velocity window so it never spans the old session; the filter re-warms
      // during the 2 s hold, so it is converged at release.
      quest_hist_count_ = 0;
      quest_vel_seeded_ = false;
      quest_vel_f_.setZero();
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

  // Live per-arm trigger feed (quest_teleop_plan §11). Published every active
  // quest tick; the solver only obeys it in TRACK, so APPROACH's held trigger
  // (which is what the engage hold needs) never moves the gripper.
  void publish_quest_trigger() {
    if (!quest_trigger_pub_) return;
    std_msgs::msg::Float32 msg;
    const int hand = (target_arm_ == "right") ? 1 : 0;   // quest_analog expects 0/1 (multiplies by QUEST_RIGHT_OFFSET)
    msg.data = static_cast<float>(quest_analog(hand, quest_trigger_analog_offset_));
    quest_trigger_pub_->publish(msg);
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

    // ── Remote control override (single bool, mirrors the Quest engage) ──
    // When TRUE the external /quest goal stream (ZMQ gateway) is authoritative:
    // suppress the SpaceMouse velocity stream at the source so the solver keeps
    // last_goal_from_quest_=true and nothing re-asserts the mapper hold pose.
    if (control_override_) {
      // The quest state machine below is frozen while the override holds (we
      // return early every tick), so nothing publishes /quest/<arm>/state and
      // the solver sits in CTRL — where update_grippers() drops the gateway's
      // /quest/<arm>/trigger. Force TRACK here so the gripper command is obeyed,
      // mirroring the physical quest engage (the pose leash is identical for
      // CTRL and TRACK, so this changes nothing else).
      if (quest_state_pub_) {
        std_msgs::msg::String state_msg;
        state_msg.data = "TRACK";
        quest_state_pub_->publish(state_msg);
      }
      return;   // gateway /quest goals drive; spacemouse + quest-trigger silent
    }

    // Left-quest grip override: runs every tick regardless of which path
    // below fills ee_goal_ (quest-engaged or SpaceMouse joystick), so holding
    // the grip re-levels the goal even if nothing else is moving it.
    apply_left_ctrl_yaw_roll_decay();

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
        // Clamps still apply on the frozen goal (soft-locks + profile).
        std::lock_guard<std::mutex> lock(ee_lock_mutex_);
        apply_ee_locks();
        apply_limit_profile();
        publish_pose();
        return;
      }
      publish_quest_trigger();   // §11: live trigger feed (gripper in TRACK)
      step_quest_interpolation();   // slerp ee_goal_ toward quest target (§6)
      {
        std::lock_guard<std::mutex> lock(ee_lock_mutex_);
        apply_ee_locks();       // soft-locks still enforced in quest mode (§6)
        apply_limit_profile();  // profile: outer hard envelope on the whole pose
      }
      publish_pose(true);   // quest active → absolute-goal topic (solver pose path)
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
    trans_scaled *= quest_precision_scale();
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
    rot_scaled *= quest_precision_scale();
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

    // Apply EE orientation locks + manual limit profile after integrating delta
    {
      std::lock_guard<std::mutex> lock(ee_lock_mutex_);
      apply_ee_locks();        // soft-locks: clamp locked roll/yaw/pitch axes
      apply_limit_profile();   // profile: outer hard envelope on the whole pose
    }

    publish_pose();
  }

  void publish_pose(bool quest = false)
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
    // quest=true → absolute-goal topic (solver's pose path); false (default) →
    // delta topic (solver's delta path). Which function the solver runs is
    // decided by which topic the last goal arrived on.
    if (quest) {
      quest_pose_pub_->publish(msg);
    } else {
      pose_pub_->publish(msg);
      publish_goal_delta();  // delta stream: one per /spacemouse pose tick
    }
  }

  void publish_goal_delta()
  {
    // Per-tick commanded delta (deltas out): what the operator commanded this
    // tick, effective in the map frame AFTER the lock/limit-profile clamps --
    // exactly what the solver recovers by differencing consecutive
    // /spacemouse/<arm>/ee_target_pose goals. linear = dx,dy,dz (m); angular =
    // drx,dry,drz (rad, extrinsic-XYZ RPY). The gateway relays it to the ZMQ
    // side as EEDelta.
    //
    // Re-base guard: mirror the solver's mapper re-base thresholds
    // (kMapperRebaseDist 0.03 m / kMapperRebaseAng 0.1 rad). A jump that size
    // is a re-base (arm switch, override release, first achieved pose, quest
    // engage/abort), not a command -- the solver adopts the new pose as a
    // baseline and never integrates it, so emit an all-zero delta and re-seed.
    constexpr double kRebaseDist = 0.03;
    constexpr double kRebaseAng = 0.1;
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    if (delta_seeded_) {
      dp = ee_goal_.translation() - delta_prev_pose_.translation();
      Eigen::Matrix3d dR = ee_goal_.linear() * delta_prev_pose_.linear().transpose();
      if (dp.norm() > kRebaseDist || Eigen::AngleAxisd(dR).angle() > kRebaseAng) {
        // Re-base: not a command. Zero the delta and re-seed below.
        dp.setZero();
      } else {
        rpy = extract_rpy(dR);
      }
    }
    delta_prev_pose_ = ee_goal_;
    delta_seeded_ = true;

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.twist.linear.x = dp.x();
    msg.twist.linear.y = dp.y();
    msg.twist.linear.z = dp.z();
    msg.twist.angular.x = rpy.x();
    msg.twist.angular.y = rpy.y();
    msg.twist.angular.z = rpy.z();
    delta_pub_->publish(msg);
  }

  // ── Subscriptions & publishers ──────────────────────────────────

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr precision_sub_;
  rclcpp::Subscription<ffw_spacemouse_msgs::msg::LeftControlOverride>::SharedPtr left_ctrl_sub_;  // /quest/left/control_override (right-arm instance only)
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_override_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ee_lock_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr limit_profile_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr achieved_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resync_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pose_changed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr quest_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr quest_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr delta_pub_;  // /spacemouse/<arm>/ee_target_delta (deltas out)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr quest_active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quest_notification_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quest_state_pub_;  // /quest/<arm>/state (§8b)
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr quest_trigger_pub_;  // /quest/<arm>/trigger (§11)
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
  // Set on machine-state resync; achieved-pose messages stamped before it are
  // stale pre-sync poses and must not anchor the re-base (epoch = no resync
  // yet, every message passes).
  rclcpp::Time resync_stamp_;

  Eigen::Isometry3d ee_goal_;
  double command_dt_ {0.01};

  // ── Delta-stream baseline (deltas out) ──────────────────────────
  // Previous ee_goal_ the per-tick delta is formed against, and whether it is
  // valid yet. Re-seeded (an all-zero delta is emitted) whenever the pose
  // stream jumps enough that the solver's mapper re-bases.
  Eigen::Isometry3d delta_prev_pose_ {Eigen::Isometry3d::Identity()};
  bool delta_seeded_ {false};

  bool precision_mode_ {false};
  bool control_override_ {false};   // remote bool gate: suppresses spacemouse stream
  double left_ctrl_trigger_ {0.0};  // left-quest trigger, 0..1 (0 = no override)
  double left_ctrl_grip_ {0.0};     // left-quest grip/side button, 0..1

  // Continuous SpaceMouse sensitivity scale driven by the left-quest trigger:
  // 1.0 when released down to quest_precision_min_scale when fully held.
  // No-op (1.0) on the left-arm instance, since left_ctrl_trigger_ is never
  // written there (left_ctrl_sub_ is only created for target_arm_ == "right").
  double quest_precision_scale() const {
    double min_scale = this->get_parameter("quest_precision_min_scale").as_double();
    return 1.0 - left_ctrl_trigger_ * (1.0 - min_scale);
  }

  // While the left-quest grip is held, slowly re-level the right SpaceMouse
  // goal's yaw and roll back to zero (pitch untouched) -- no-op (grip stays
  // 0.0) on the left-arm instance, same guard as quest_precision_scale().
  static constexpr double kLeftCtrlYawRollDecay = 0.98;
  void apply_left_ctrl_yaw_roll_decay() {
    if (left_ctrl_grip_ <= 0.5) return;
    Eigen::Vector3d rpy = extract_rpy(ee_goal_.linear());
    double roll = rpy.x() * kLeftCtrlYawRollDecay;
    double pitch = rpy.y();
    double yaw = rpy.z() * kLeftCtrlYawRollDecay;
    ee_goal_.linear() = rpy_to_matrix(roll, pitch, yaw);
  }

  // ── EE lock state ───────────────────────────────────────────────
  std::mutex ee_lock_mutex_;
  bool soft_lock_ee_roll_ {false};
  bool soft_lock_ee_yaw_ {false};
  bool soft_lock_ee_pitch_ {false};
  double soft_locked_roll_ {0.0};
  double soft_locked_yaw_ {0.0};
  double soft_locked_pitch_ {0.0};
  double ee_orientation_slack_ {0.0};  // set from parameter ee_orientation_slack_deg

  // ── Manual limit profile (CLI-created; guarded by ee_lock_mutex_) ──
  LimitProfile limit_profile_;

  // ── Quest override state (quest_teleop_plan §4-§10, Task 2) ──
  QuestState quest_state_ {QuestState::SM_CONTROL};   // quest_active_ ⇔ != SM_CONTROL
  bool engage_armed_ {false};      // §4: count arms only from all-four-zero
  bool hold_started_ {false};
  rclcpp::Time hold_start_time_;
  rclcpp::Time quest_last_msg_time_;      // §9 arrival watchdog
  std::vector<float> quest_data_;         // cached /quest_state flat array
  bool quest_data_valid_ {false};
  bool quest_debug_engage_ {false};       // §4 debug: log detect_engage edges
  std::string last_debug_engage_label_;   // transition-only printing
  double last_debug_progress_s_ {-1.0};   // §4 debug: HOLDING countdown tick
  double last_debug_live_s_ {-1.0};       // §4 debug: LIVE value stream throttle
  bool quest_debug_error_ {false};        // §10 debug: print APPROACH error budget
  double last_debug_error_s_ {-1.0};      // §10 debug: error print throttle
  Eigen::Isometry3d quest_target_ {Eigen::Isometry3d::Identity()};  // live interpolation target

  // Cached quest parameters
  std::string quest_state_topic_ {"/quest_state"};
  std::string quest_active_topic_;
  std::string quest_state_pub_topic_;   // /quest/<arm>/state — per-state leash feed (§8b)
  std::string quest_trigger_pub_topic_;   // /quest/<arm>/trigger — gripper feed (§11)
  std::string quest_notification_topic_ {"/ffw_control/notification"};
  bool quest_publish_notifications_ {true};
  double quest_hold_engage_s_ {2.0};
  double quest_release_min_ {0.1};
  int quest_thumb_analog_offset_ {7};
  int quest_trigger_analog_offset_ {6};
  double quest_thumb_engage_min_ {0.5};
  double quest_trigger_engage_min_ {0.5};
  double quest_w_slow_ {0.15};
  double quest_approach_pos_m_ {0.05};
  double quest_approach_ang_rad_ {0.10};
  double quest_timeout_s_ {0.5};
  double quest_achieved_max_age_s_ {0.5};
  double quest_hold_hysteresis_ {0.05};
  Eigen::Isometry3d quest_frame_rot_ {Eigen::Isometry3d::Identity()};  // optional alignment
  double quest_pos_scale_ {1.0};
  double quest_xy_scale_ {3.0};
  double quest_lead_t_ {0.25};     // §6 TEST 3: lead distance = hand velocity × T_lead
  double quest_vel_alpha_ {0.2};   // §6 TEST 3: EMA factor on the smoothed velocity

  // §6 TEST 3: hand-velocity history for the feedforward lead. Parallel ring
  // buffers of (time, position); the windowed secant over the N samples equals
  // the least-squares slope for constant-velocity motion, so the estimate is
  // far steadier than the per-tick Δh.
  static constexpr size_t kQuestHistN = 5;
  std::array<rclcpp::Time, kQuestHistN> quest_hist_t_;
  std::array<Eigen::Vector3d, kQuestHistN> quest_hist_p_;
  size_t quest_hist_head_ {0};
  size_t quest_hist_count_ {0};
  Eigen::Vector3d quest_vel_f_ {Eigen::Vector3d::Zero()};  // EMA-smoothed velocity (m/s)
  bool quest_vel_seeded_ {false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyHand>());
  rclcpp::shutdown();
  return 0;
}
