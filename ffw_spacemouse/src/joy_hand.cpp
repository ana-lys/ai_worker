#include <algorithm>
#include <chrono>
#include <cmath>
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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ffw_spacemouse_msgs/msg/left_control_override.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ffw_collision_checker/msg/goal_constraints.hpp"

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

    // Global limit profile: ee_goal_'s assumed native frame, used as the
    // source frame when TF-looking-up into a global profile's target frame
    // (e.g. an AprilTag "marker_frame" published by ffw_odom).
    this->declare_parameter("ee_goal_frame", "base_link");
    ee_goal_frame_ = this->get_parameter("ee_goal_frame").as_string();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter("quest_achieved_pose_topic_l", "/ik_solver/achieved_ee_pose_l");
    this->declare_parameter("quest_achieved_pose_topic_r", "/ik_solver/achieved_ee_pose_r");

    std::string topic_name = "/spacemouse/" + target_arm_ + "/ee_target_pose";
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);

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
          if (msg->data == "ARM" && current_mode_ != "ARM") {
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

    // ── Achieved-pose subscription (quest_teleop_plan §5/§11, Task 1) ──
    // World-frame EE pose from the solver, used to re-base the mapper's delta
    // accumulation onto the real pose. No new
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

    // ── External hold (script-owned arm) ──────────────────────────
    // A script (e.g. contact_force_episodes.py) drives this arm through the
    // solver's absolute-pose topic /quest/<arm>/ee_target_pose. Our idle
    // 100 Hz delta stream would pull the solver target back to our stale
    // ee_goal_, so we go silent while held. `true` must be re-sent at least
    // every external_hold_timeout_s; release is `false` or that timeout.
    // Release re-bases ee_goal_ onto the achieved pose (see control timer).
    this->declare_parameter("external_hold_timeout_s", 1.0);
    external_hold_timeout_s_ = this->get_parameter("external_hold_timeout_s").as_double();
    external_hold_last_true_ = this->now();
    external_hold_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/spacemouse/" + target_arm_ + "/external_hold", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            external_hold_last_true_ = this->now();
            if (!external_hold_) {
              external_hold_ = true;
              RCLCPP_INFO(this->get_logger(),
                "External hold ON — mapper silent (%s)", target_arm_.c_str());
            }
          } else if (external_hold_) {
            release_external_hold("released");
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
        if (action == "clearg") {
          global_limit_profile_.active = false;
          RCLCPP_INFO(this->get_logger(), "Global limit profile cleared (%s)", target_arm_.c_str());
          return;
        }
        if (action == "setg") {
          // Format: "setg <arm> <frame> <12 bounds>" -- same 12-bound layout
          // as "set", plus a leading frame token naming the TF frame the box
          // was recorded in (e.g. "marker_frame").
          std::string frame;
          ss >> frame;
          if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(),
              "Invalid limit_profile setg format — missing frame token");
            return;
          }
          double bounds[12];
          for (double &b : bounds) {
            if (!(ss >> b)) {
              RCLCPP_WARN(this->get_logger(),
                "Invalid limit_profile setg format — need frame + 12 bounds, got partial parse");
              return;
            }
          }
          GlobalLimitProfile &gp = global_limit_profile_;
          gp.frame_id = frame;
          gp.px_min = bounds[0]; gp.px_max = bounds[1];
          gp.py_min = bounds[2]; gp.py_max = bounds[3];
          gp.pz_min = bounds[4]; gp.pz_max = bounds[5];
          gp.roll_min = bounds[6]; gp.roll_max = bounds[7];
          gp.pitch_min = bounds[8]; gp.pitch_max = bounds[9];
          gp.yaw_min = bounds[10]; gp.yaw_max = bounds[11];
          gp.active = true;
          RCLCPP_INFO(this->get_logger(),
            "Global limit profile set (%s, frame=%s): x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] "
            "roll[%.3f,%.3f] pitch[%.3f,%.3f] yaw[%.3f,%.3f]",
            target_arm_.c_str(), gp.frame_id.c_str(), gp.px_min, gp.px_max, gp.py_min, gp.py_max,
            gp.pz_min, gp.pz_max, gp.roll_min, gp.roll_max, gp.pitch_min, gp.pitch_max,
            gp.yaw_min, gp.yaw_max);
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

    // Live goal constraints (EE locks + limit boxes) for ffw_ik_shadow_solver,
    // which clamps its query goals the same way (goal_constraints.hpp).
    // Latched so a late-starting shadow gets the current state; polled and
    // republished on change so every mutation site (callbacks, resync
    // re-capture) is covered.
    goal_constraints_pub_ = this->create_publisher<ffw_collision_checker::msg::GoalConstraints>(
      "/spacemouse/" + target_arm_ + "/goal_constraints",
      rclcpp::QoS(1).reliable().transient_local());
    goal_constraints_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() {
        ffw_collision_checker::msg::GoalConstraints msg;
        {
          std::lock_guard<std::mutex> lock(ee_lock_mutex_);
          msg.arm = target_arm_;
          msg.goal_frame = ee_goal_frame_;
          msg.roll_locked = soft_lock_ee_roll_;
          msg.pitch_locked = soft_lock_ee_pitch_;
          msg.yaw_locked = soft_lock_ee_yaw_;
          msg.locked_roll = soft_locked_roll_;
          msg.locked_pitch = soft_locked_pitch_;
          msg.locked_yaw = soft_locked_yaw_;
          msg.orientation_slack = ee_orientation_slack_;
          const LimitProfile &p = limit_profile_;
          msg.local_active = p.active;
          msg.local_bounds = {p.px_min, p.px_max, p.py_min, p.py_max, p.pz_min, p.pz_max,
                              p.roll_min, p.roll_max, p.pitch_min, p.pitch_max, p.yaw_min, p.yaw_max};
          const GlobalLimitProfile &g = global_limit_profile_;
          msg.global_active = g.active;
          msg.global_frame = g.frame_id;
          msg.global_bounds = {g.px_min, g.px_max, g.py_min, g.py_max, g.pz_min, g.pz_max,
                               g.roll_min, g.roll_max, g.pitch_min, g.pitch_max, g.yaw_min, g.yaw_max};
        }
        if (goal_constraints_published_ && msg == last_goal_constraints_) return;
        goal_constraints_pub_->publish(msg);
        last_goal_constraints_ = msg;
        goal_constraints_published_ = true;
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

  static Eigen::Matrix3d rpy_to_matrix(double roll, double pitch, double yaw) {
    // Extrinsic XYZ Euler: R = Rx(roll) * Ry(pitch) * Rz(yaw)
    return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix() *
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }

  // Hand-rolled TransformStamped -> Eigen::Isometry3d, consistent with this
  // file's existing hand-rolled quaternion/rotation-matrix style (avoids
  // adding a tf2_eigen dependency for one conversion).
  static Eigen::Isometry3d transform_to_eigen(const geometry_msgs::msg::TransformStamped &t) {
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = Eigen::Vector3d(
      t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    Eigen::Quaterniond q(t.transform.rotation.w, t.transform.rotation.x,
                         t.transform.rotation.y, t.transform.rotation.z);
    out.linear() = q.normalized().toRotationMatrix();
    return out;
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

  // ── Global limit profile (CLI-created box expressed in an external TF
  // frame, e.g. an AprilTag "marker_frame"). Same shape as LimitProfile plus
  // frame_id so apply_global_limit_profile() knows what to TF-lookup against.
  // Guarded by ee_lock_mutex_, same contract as limit_profile_.
  struct GlobalLimitProfile {
    bool active {false};
    std::string frame_id;  // target frame the box was recorded in, e.g. "marker_frame"
    double px_min {0}, px_max {0};
    double py_min {0}, py_max {0};
    double pz_min {0}, pz_max {0};
    double roll_min {0}, roll_max {0};
    double pitch_min {0}, pitch_max {0};
    double yaw_min {0}, yaw_max {0};
  };

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

  void apply_global_limit_profile() {
    // Same contract as apply_limit_profile(): caller holds ee_lock_mutex_,
    // call AFTER apply_ee_locks() and apply_limit_profile() so this is the
    // outermost envelope. Fails OPEN (no clamp) if the target frame isn't
    // currently broadcast -- never crashes, never clamps on stale/bad data.
    if (!global_limit_profile_.active) return;

    geometry_msgs::msg::TransformStamped t_lookup;
    try {
      t_lookup = tf_buffer_->lookupTransform(
        global_limit_profile_.frame_id, ee_goal_frame_,
        tf2::TimePointZero, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Global limit profile (%s): TF lookup %s -> %s failed (%s) -- "
        "not clamping this tick", target_arm_.c_str(),
        ee_goal_frame_.c_str(), global_limit_profile_.frame_id.c_str(), ex.what());
      return;
    }

    const Eigen::Isometry3d T_frame_from_goal = transform_to_eigen(t_lookup);
    Eigen::Isometry3d goal_in_frame = T_frame_from_goal * ee_goal_;

    const GlobalLimitProfile &p = global_limit_profile_;
    Eigen::Vector3d tr = goal_in_frame.translation();
    tr.x() = std::clamp(tr.x(), p.px_min, p.px_max);
    tr.y() = std::clamp(tr.y(), p.py_min, p.py_max);
    tr.z() = std::clamp(tr.z(), p.pz_min, p.pz_max);
    goal_in_frame.translation() = tr;

    Eigen::Vector3d rpy = extract_rpy(goal_in_frame.linear());
    double roll = std::clamp(rpy.x(), p.roll_min, p.roll_max);
    double pitch = std::clamp(rpy.y(), p.pitch_min, p.pitch_max);
    double yaw = std::clamp(rpy.z(), p.yaw_min, p.yaw_max);
    goal_in_frame.linear() = rpy_to_matrix(roll, pitch, yaw);

    // Transform back into ee_goal_'s native frame.
    ee_goal_ = T_frame_from_goal.inverse() * goal_in_frame;
  }

  // ── Callbacks ───────────────────────────────────────────────────

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    latest_joy_ = *msg;
    joy_received_ = true;
  }

  // End an external hold: re-base ee_goal_ onto where the arm actually is, so
  // the first delta after release carries no jump back to the pre-hold goal.
  void release_external_hold(const char *why)
  {
    external_hold_ = false;
    if (achieved_pose_valid_) ee_goal_ = achieved_pose_map_;
    RCLCPP_INFO(this->get_logger(),
      "External hold OFF (%s) — ee_goal_ re-based to achieved pose (%s)",
      why, target_arm_.c_str());
  }

  void control_timer_callback()
  {
    if (current_mode_ == "BASE") return;

    if (external_hold_) {
      if ((this->now() - external_hold_last_true_).seconds() > external_hold_timeout_s_) {
        release_external_hold("timeout");
      } else {
        return;   // script owns the arm: publish nothing
      }
    }

    // Left-quest grip override: runs every tick before the joy_received_
    // gate, so holding the grip re-levels the goal even if nothing else is
    // moving it.
    apply_left_ctrl_yaw_roll_decay();

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
      apply_global_limit_profile();  // global: outer envelope in an external (e.g. marker) frame
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
    publish_goal_delta();  // delta stream: one per /spacemouse pose tick
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
    // is a re-base (arm switch, override release, first achieved pose), not a command -- the solver adopts the new pose as a
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ee_lock_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr limit_profile_sub_;
  rclcpp::Publisher<ffw_collision_checker::msg::GoalConstraints>::SharedPtr goal_constraints_pub_;
  rclcpp::TimerBase::SharedPtr goal_constraints_timer_;
  ffw_collision_checker::msg::GoalConstraints last_goal_constraints_;
  bool goal_constraints_published_ {false};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr achieved_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resync_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pose_changed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr delta_pub_;  // /spacemouse/<arm>/ee_target_delta (deltas out)
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string target_arm_;
  std::mutex joy_mutex_;
  sensor_msgs::msg::Joy latest_joy_;
  bool joy_received_ {false};
  std::string current_mode_ {"BASE"};

  // ── World-frame re-base state (quest_teleop_plan §11, Task 1) ──
  Eigen::Isometry3d achieved_pose_map_ {Eigen::Isometry3d::Identity()};
  bool achieved_pose_valid_ {false};
  // Set on machine-state resync; achieved-pose messages stamped before it are
  // stale pre-sync poses and must not anchor the re-base (epoch = no resync
  // yet, every message passes).
  rclcpp::Time resync_stamp_;

  // External hold (/spacemouse/<arm>/external_hold): while held, an external
  // script owns this arm's goal via the solver's absolute-pose topic and this
  // mapper publishes nothing. Expires external_hold_timeout_s after the last
  // `true`, so a dead script can't lock the SpaceMouse out.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr external_hold_sub_;
  bool external_hold_ {false};
  rclcpp::Time external_hold_last_true_;
  double external_hold_timeout_s_ {1.0};

  Eigen::Isometry3d ee_goal_;
  double command_dt_ {0.01};

  // ── Delta-stream baseline (deltas out) ──────────────────────────
  // Previous ee_goal_ the per-tick delta is formed against, and whether it is
  // valid yet. Re-seeded (an all-zero delta is emitted) whenever the pose
  // stream jumps enough that the solver's mapper re-bases.
  Eigen::Isometry3d delta_prev_pose_ {Eigen::Isometry3d::Identity()};
  bool delta_seeded_ {false};

  bool precision_mode_ {false};
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

  // ── Global limit profile (CLI-created; frame from an external TF source,
  // e.g. an AprilTag marker board via ffw_odom's marker_frame_broadcaster) ──
  GlobalLimitProfile global_limit_profile_;
  std::string ee_goal_frame_;  // assumed native frame of ee_goal_; from param "ee_goal_frame"
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyHand>());
  rclcpp::shutdown();
  return 0;
}
