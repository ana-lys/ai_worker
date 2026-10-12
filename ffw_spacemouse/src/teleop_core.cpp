// teleop_core: the only publisher of the arm teleop goals to the IK solver
// (MARKER_TELEOP_PLAN.md 0b, the arbiter minus leases).
//
// Each arm's goal arrives from joy_hand (use_teleop_core:=true) on
// /teleop_core/<arm>/ee_goal in its native frame (header.frame_id):
//   - base_frame / "map" goals are relayed to the solver as they arrive
//     (same stream as joy_hand publishing directly);
//   - marker_frame goals are stored and re-expressed in the base frame every
//     tick with ONE T_base<-marker lookup shared by both arms (D3), so they
//     stay fixed on the board while the base moves -- in BASE mode too,
//     where joy_hand is silent.
// /spacemouse/<arm>/external_hold silences an arm exactly like joy_hand.
//
// Base: the only publisher of /cmd_vel. joy_base and the Logitech send their
// twists to /teleop_core/base/{spacemouse,logitech}. The SpaceMouse has
// priority; the Logitech passes only once the SpaceMouse twist has been zero
// (inside its deadband) or absent for spacemouse_idle_s (D6). While any goal
// is anchored the whole twist is scaled by ONE factor to the anchored speed
// cap (one factor keeps the swerve steering targets fixed). No fresh input:
// one zero, then silence, so other /cmd_vel users are not fought.
//
// Marker health (marker_health:=true, D4): HALT when no board detection for
// > 2 s, marker TF older than marker_max_age_s, or (while goals are
// anchored) one EKF correction jump > 5 cm / 3 deg beyond the commanded base
// motion. HALT = base twist forced to zero, anchored goals frozen at their
// last base-frame pose, incoming goals dropped; latched /teleop_core/halted
// tells joy_hand to hold. Resume after 1 s healthy: frozen goals re-anchored
// in marker_frame where they are. Starts halted.
//
// Base clamp while goals are anchored (D5), on top of the speed cap, always
// as ONE factor on the whole twist:
//   predictive: scale the twist so no anchored goal, carried clamp_horizon_s
//     ahead, leaves its arm's reach box (base frame), or moves further
//     outside it if it already is;
//   reactive: if an arm's achieved EE is > 2 cm / 3 deg from its goal while
//     the base moves, only error-reducing twists pass until both arms are
//     back under 1 cm / 1.5 deg.
//
// Lift (D7, BASE mode only, like the solver's /left/joy lift): dz requests
// (m) on /teleop_core/lift/{spacemouse,logitech}, applied at <=
// lift_max_speed (0.1 m/s, the solver's own SpaceMouse lift rate). The
// SpaceMouse is a rate input: what arrives in a tick is applied up to that
// step and the excess dropped (no backlog after release). Logitech requests
// are targets: queued and ramped while the SpaceMouse lift is idle. The
// shift moves every stored goal vertically in the base frame
// (anchored goals stay stored in marker_frame) so the EEs ride the lift; the
// ramp keeps each step under the solver's 3 cm re-base threshold. Anchored
// goals never leave their reach box in z.
//
// Head: trajectories on /teleop_core/head/{spacemouse,logitech} are relayed
// to head_topic; the Logitech's only while the SpaceMouse head has been
// quiet for spacemouse_idle_s.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <deque>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose &p)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  out.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y,
                                    p.orientation.z).normalized().toRotationMatrix();
  return out;
}

Eigen::Isometry3d transform_to_eigen(const geometry_msgs::msg::TransformStamped &t)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  out.linear() = Eigen::Quaterniond(t.transform.rotation.w, t.transform.rotation.x,
                                    t.transform.rotation.y, t.transform.rotation.z)
                   .normalized().toRotationMatrix();
  return out;
}

geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d &T)
{
  geometry_msgs::msg::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  const Eigen::Quaterniond q(T.linear());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

}  // namespace

class TeleopCore : public rclcpp::Node
{
public:
  TeleopCore() : Node("teleop_core")
  {
    base_frame_ = this->declare_parameter("base_frame", "base_link");
    marker_frame_ = this->declare_parameter("marker_frame", "marker_frame");
    marker_max_age_s_ = this->declare_parameter("marker_max_age_s", 0.3);
    external_hold_timeout_s_ = this->declare_parameter("external_hold_timeout_s", 1.0);
    const double rate = std::max(1.0, this->declare_parameter("rate_hz", 100.0));
    tick_dt_ = 1.0 / rate;
    base_cmd_timeout_s_ = this->declare_parameter("base_cmd_timeout_s", 0.3);
    spacemouse_idle_s_ = this->declare_parameter("spacemouse_idle_s", 0.3);
    anchored_max_linear_ = this->declare_parameter("anchored_max_linear_vel", 0.10);
    anchored_max_angular_ = this->declare_parameter("anchored_max_angular_vel", 0.20);

    // Reach boxes in the base frame [x_min x_max y_min y_max z_min z_max].
    // Defaults are rough (shoulders at y +-0.23 m, ~0.8 m reach; z depends
    // on the lift) -- tune per setup.
    const auto box_l = this->declare_parameter("reach_box_left",
      std::vector<double>{0.15, 0.75, -0.10, 0.60, 0.40, 1.60});
    const auto box_r = this->declare_parameter("reach_box_right",
      std::vector<double>{0.15, 0.75, -0.60, 0.10, 0.40, 1.60});
    clamp_horizon_s_ = this->declare_parameter("clamp_horizon_s", 0.3);
    trip_m_ = this->declare_parameter("reactive_trip_m", 0.02);
    trip_rad_ = this->declare_parameter("reactive_trip_deg", 3.0) * M_PI / 180.0;
    clear_m_ = this->declare_parameter("reactive_clear_m", 0.01);
    clear_rad_ = this->declare_parameter("reactive_clear_deg", 1.5) * M_PI / 180.0;
    achieved_max_age_s_ = this->declare_parameter("achieved_max_age_s", 0.5);

    marker_health_ = this->declare_parameter("marker_health", false);
    detection_timeout_s_ = this->declare_parameter("detection_timeout_s", 2.0);
    resume_healthy_s_ = this->declare_parameter("resume_healthy_s", 1.0);
    jump_max_m_ = this->declare_parameter("jump_max_m", 0.05);
    jump_max_rad_ = this->declare_parameter("jump_max_deg", 3.0) * M_PI / 180.0;
    warn_min_rate_hz_ = this->declare_parameter("detection_warn_rate_hz", 6.4);
    warn_max_gap_s_ = this->declare_parameter("detection_warn_gap_s", 0.6);
    halted_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/teleop_core/halted", rclcpp::QoS(1).reliable().transient_local());
    if (marker_health_) {
      detection_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->declare_parameter("detection_topic", "/head_camera/marker_board_pose"),
        rclcpp::QoS(5).best_effort(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr) {
          detections_.push_back(this->now());
          last_detection_ = detections_.back();
          have_detection_ = true;
        });
      set_halted(true, "starting suspended until marker health is OK");
    } else {
      set_halted(false, nullptr);
    }

    sources_[kSpaceMouse].name = "spacemouse";
    sources_[kLogitech].name = "logitech";
    for (BaseSource &src : sources_) {
      src.sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/teleop_core/base/" + src.name, 10,
        [this, &src](const geometry_msgs::msg::Twist::SharedPtr msg) {
          src.twist = *msg;
          src.stamp = this->now();
          src.received = true;
        });
    }
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->declare_parameter("cmd_vel_topic", "/cmd_vel"), 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    lift_max_speed_ = this->declare_parameter("lift_max_speed", 0.10);
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/teleop_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        mode_ = msg->data;
        if (mode_ != "BASE") pending_lift_sm_ = pending_lift_lg_ = 0.0;
      });
    for (const std::string src : {"spacemouse", "logitech"}) {
      lift_subs_.push_back(this->create_subscription<std_msgs::msg::Float64>(
        "/teleop_core/lift/" + src, 10, [this, src](const std_msgs::msg::Float64::SharedPtr msg) {
          if (mode_ != "BASE" || halted_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "Lift from %s ignored (%s)", src.c_str(), halted_ ? "halted" : "ARM mode");
            return;
          }
          (src == "spacemouse" ? pending_lift_sm_ : pending_lift_lg_) += msg->data;
        }));
    }
    head_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->declare_parameter("head_topic", "/leader/joystick_controller_left/joint_trajectory"), 10);
    head_sm_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/teleop_core/head/spacemouse", 10,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        head_sm_last_ = this->now();
        head_sm_seen_ = true;
        head_pub_->publish(*msg);
      });
    head_lg_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/teleop_core/head/logitech", 10,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        if (head_sm_seen_ && (this->now() - head_sm_last_).seconds() < spacemouse_idle_s_) return;
        head_pub_->publish(*msg);
      });

    const std::array<std::string, 2> names {"left", "right"};
    const std::array<std::vector<double>, 2> boxes {box_l, box_r};
    for (size_t i = 0; i < arms_.size(); ++i) {
      Arm &arm = arms_[i];
      arm.name = names[i];
      if (boxes[i].size() != 6) {
        throw std::invalid_argument("reach_box_" + arm.name + " needs 6 values");
      }
      for (int k = 0; k < 3; ++k) {
        arm.box_min[k] = boxes[i][2 * k];
        arm.box_max[k] = boxes[i][2 * k + 1];
      }
      arm.achieved_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        std::string("/ik_solver/achieved_ee_pose_") + arm.name[0], 10,
        [this, &arm](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          arm.achieved = pose_to_eigen(msg->pose);
          arm.achieved_stamp = this->now();
          arm.achieved_valid = true;
        });
      arm.goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/spacemouse/" + arm.name + "/ee_target_pose", 10);
      arm.goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/teleop_core/" + arm.name + "/ee_goal", 10,
        [this, &arm](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          on_goal(arm, *msg);
        });
      arm.hold_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/spacemouse/" + arm.name + "/external_hold", 10,
        [this, &arm](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            arm.hold_last_true = this->now();
            if (!arm.held) {
              arm.held = true;
              arm.valid = false;   // stale once the script moves the arm
              RCLCPP_INFO(this->get_logger(), "External hold ON (%s)", arm.name.c_str());
            }
          } else if (arm.held) {
            arm.held = false;
            RCLCPP_INFO(this->get_logger(), "External hold OFF (%s)", arm.name.c_str());
          }
        });
    }

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      std::bind(&TeleopCore::tick, this));

    RCLCPP_INFO(this->get_logger(), "teleop_core: %s goals anchored via TF %s <- %s",
      marker_frame_.c_str(), base_frame_.c_str(), marker_frame_.c_str());
  }

private:
  struct Arm
  {
    std::string name;
    bool valid {false};
    bool anchored {false};            // goal frame is marker_frame
    bool frozen {false};              // anchored goal frozen in the base frame by a HALT
    Eigen::Vector3d box_min, box_max;  // reach box, base frame
    Eigen::Isometry3d achieved {Eigen::Isometry3d::Identity()};   // solver's EE, base frame
    rclcpp::Time achieved_stamp;
    bool achieved_valid {false};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr achieved_sub;
    Eigen::Isometry3d goal {Eigen::Isometry3d::Identity()};   // in its native frame
    bool held {false};
    rclcpp::Time hold_last_true;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub;
  };

  struct BaseSource
  {
    std::string name;
    geometry_msgs::msg::Twist twist;
    rclcpp::Time stamp;
    bool received {false};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
  };
  enum { kSpaceMouse = 0, kLogitech = 1 };

  void on_goal(Arm &arm, const geometry_msgs::msg::PoseStamped &msg)
  {
    const std::string &frame = msg.header.frame_id;
    if (frame == marker_frame_) {
      arm.anchored = true;
    } else if (frame == base_frame_ || frame == "map") {
      arm.anchored = false;
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "%s goal in unsupported frame '%s' -- dropped", arm.name.c_str(), frame.c_str());
      return;
    }
    if (hold_active(arm) || halted_) return;
    arm.goal = pose_to_eigen(msg.pose);
    arm.valid = true;
    if (!arm.anchored) publish_goal(arm, arm.goal);   // relay as it arrives
  }

  bool hold_active(Arm &arm)
  {
    if (arm.held && (this->now() - arm.hold_last_true).seconds() > external_hold_timeout_s_) {
      arm.held = false;
      RCLCPP_INFO(this->get_logger(), "External hold OFF (timeout, %s)", arm.name.c_str());
    }
    return arm.held;
  }

  // The single T_base<-marker lookup of this tick (D3). false = missing or
  // older than marker_max_age_s: anchored goals hold (nothing published).
  bool lookup_base_from_marker(Eigen::Isometry3d &T)
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(base_frame_, marker_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s -> %s unavailable (%s) -- anchored goals hold", marker_frame_.c_str(),
        base_frame_.c_str(), ex.what());
      return false;
    }
    const double age = (this->now() - rclcpp::Time(t.header.stamp)).seconds();
    if (age > marker_max_age_s_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s -> %s is %.2f s old -- anchored goals hold", marker_frame_.c_str(),
        base_frame_.c_str(), age);
      return false;
    }
    T = transform_to_eigen(t);
    tf_stamp_ = rclcpp::Time(t.header.stamp);
    return true;
  }

  void set_halted(bool halted, const char *why)
  {
    halted_ = halted;
    std_msgs::msg::Bool msg;
    msg.data = halted;
    halted_pub_->publish(msg);
    if (why) {
      if (halted) RCLCPP_WARN(this->get_logger(), "HALT: %s", why);
      else RCLCPP_INFO(this->get_logger(), "Resumed: %s", why);
    }
  }

  // EKF correction jump: new TF vs the previous one carried forward by the
  // commanded base motion (T_base'<-marker = D^-1 * T_base<-marker).
  bool ekf_jumped(const Eigen::Isometry3d &T_now, std::string &why)
  {
    const bool fresh_tf = !have_prev_tf_ || tf_stamp_ != prev_tf_stamp_;
    if (!fresh_tf) return false;
    bool jumped = false;
    if (have_prev_tf_) {
      const double dt = (tf_stamp_ - prev_tf_stamp_).seconds();
      const double wz = last_out_.angular.z * dt;
      Eigen::Isometry3d D = Eigen::Isometry3d::Identity();
      D.translation() = Eigen::Vector3d(last_out_.linear.x * dt, last_out_.linear.y * dt, 0.0);
      D.linear() = Eigen::AngleAxisd(wz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const Eigen::Isometry3d err = (D.inverse() * prev_tf_).inverse() * T_now;
      const double dp = err.translation().norm();
      const double da = Eigen::AngleAxisd(err.linear()).angle();
      if (dp > jump_max_m_ || da > jump_max_rad_) {
        jumped = true;
        why = "EKF jump " + std::to_string(dp * 100.0).substr(0, 4) + " cm / " +
              std::to_string(da * 180.0 / M_PI).substr(0, 4) + " deg";
      }
    }
    prev_tf_ = T_now;
    prev_tf_stamp_ = tf_stamp_;
    have_prev_tf_ = true;
    return jumped;
  }

  // One health evaluation per tick (D4). Returns the reason when unhealthy.
  bool marker_healthy(bool tf_ok, const Eigen::Isometry3d &T, bool check_jump, std::string &why)
  {
    const rclcpp::Time now = this->now();
    while (!detections_.empty() && (now - detections_.front()).seconds() > 2.0)
      detections_.pop_front();
    // WARN only: low rate / long gap inside the 2 s window
    const double rate = detections_.size() / 2.0;
    double gap = 0.0;
    for (size_t i = 1; i < detections_.size(); ++i)
      gap = std::max(gap, (detections_[i] - detections_[i - 1]).seconds());
    if (!halted_ && (rate < warn_min_rate_hz_ || gap > warn_max_gap_s_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Board detections weak: %.1f Hz, max gap %.2f s (2 s window)", rate, gap);
    }
    const double det_age = have_detection_ ? (now - last_detection_).seconds() : 1e9;
    if (det_age > detection_timeout_s_) {
      why = "no board detection for > " + std::to_string(detection_timeout_s_).substr(0, 3) + " s";
      return false;
    }
    if (!tf_ok) {
      why = marker_frame_ + " TF missing or stale (EKF dead?)";
      have_prev_tf_ = false;
      return false;
    }
    if (check_jump && ekf_jumped(T, why)) return false;
    if (!check_jump) {
      prev_tf_ = T;
      prev_tf_stamp_ = tf_stamp_;
      have_prev_tf_ = true;
    }
    return true;
  }

  void enter_halt(const std::string &why)
  {
    for (Arm &arm : arms_) {
      if (arm.valid && arm.anchored && have_last_good_tf_) {
        arm.goal = last_good_tf_ * arm.goal;   // freeze at the last published base pose
        arm.anchored = false;
        arm.frozen = true;
      }
    }
    set_halted(true, why.c_str());
  }

  void resume(const Eigen::Isometry3d &T_base_marker)
  {
    for (Arm &arm : arms_) {
      if (arm.frozen) {
        arm.goal = T_base_marker.inverse() * arm.goal;   // re-anchor where it is
        arm.anchored = true;
        arm.frozen = false;
      }
    }
    set_halted(false, "marker healthy");
  }

  void tick()
  {
    bool any_anchored = false;
    for (Arm &arm : arms_) {
      if (arm.valid && arm.anchored && !hold_active(arm)) any_anchored = true;
    }
    Eigen::Isometry3d T_base_marker = Eigen::Isometry3d::Identity();
    const bool tf_ok =
      (any_anchored || marker_health_) && lookup_base_from_marker(T_base_marker);

    if (marker_health_) {
      std::string why;
      const bool healthy = marker_healthy(tf_ok, T_base_marker, any_anchored, why);
      const rclcpp::Time now = this->now();
      if (!healthy) {
        healthy_since_valid_ = false;
        if (!halted_) enter_halt(why);
        else RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Halted: %s", why.c_str());
      } else if (halted_) {
        if (!healthy_since_valid_) {
          healthy_since_ = now;
          healthy_since_valid_ = true;
        } else if ((now - healthy_since_).seconds() >= resume_healthy_s_) {
          resume(T_base_marker);
          any_anchored = true;
        }
      }
    }

    step_lift(any_anchored, tf_ok, T_base_marker);

    if (tf_ok && !halted_) {
      last_good_tf_ = T_base_marker;
      have_last_good_tf_ = true;
      for (Arm &arm : arms_) {
        if (arm.valid && arm.anchored && !arm.held) publish_goal(arm, T_base_marker * arm.goal);
      }
    }
    step_base(any_anchored || halted_, (tf_ok && !halted_) ? &T_base_marker : nullptr);
  }

  // One ramp step of the pending lift. Anchored goals need this tick's TF.
  void step_lift(bool any_anchored, bool tf_ok, const Eigen::Isometry3d &T_bm)
  {
    if (halted_) pending_lift_sm_ = pending_lift_lg_ = 0.0;
    if (pending_lift_sm_ == 0.0 && pending_lift_lg_ == 0.0) return;
    if (any_anchored && !tf_ok) {   // no board: drop the rate input, keep targets
      pending_lift_sm_ = 0.0;
      return;
    }
    const double max_step = lift_max_speed_ * tick_dt_;
    const bool from_sm = pending_lift_sm_ != 0.0;
    double dz = std::clamp(from_sm ? pending_lift_sm_ : pending_lift_lg_, -max_step, max_step);
    pending_lift_sm_ = 0.0;
    // Anchored goals stay inside their reach box in z (no further outside).
    for (const Arm &arm : arms_) {
      if (!(arm.valid && arm.anchored && !arm.held)) continue;
      const double z = (T_bm * arm.goal).translation().z();
      if (dz > 0.0) dz = std::min(dz, std::max(0.0, arm.box_max.z() - z));
      else dz = std::max(dz, std::min(0.0, arm.box_min.z() - z));
    }
    if (dz == 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Lift stopped: an anchored goal is at its reach box z limit");
      pending_lift_lg_ = 0.0;
      return;
    }
    if (!from_sm) {
      pending_lift_lg_ -= dz;
      if (std::abs(pending_lift_lg_) < 1e-9) pending_lift_lg_ = 0.0;
    }
    Eigen::Isometry3d up = Eigen::Isometry3d::Identity();
    up.translation().z() = dz;
    for (Arm &arm : arms_) {
      if (!arm.valid || arm.held) continue;
      if (arm.anchored) {
        arm.goal = T_bm.inverse() * up * T_bm * arm.goal;   // published below
      } else {
        arm.goal = up * arm.goal;
        publish_goal(arm, arm.goal);
      }
    }
  }

  static bool is_zero(const geometry_msgs::msg::Twist &t)
  {
    return t.linear.x == 0.0 && t.linear.y == 0.0 && t.linear.z == 0.0 &&
           t.angular.x == 0.0 && t.angular.y == 0.0 && t.angular.z == 0.0;
  }

  bool fresh(const BaseSource &src, const rclcpp::Time &now) const
  {
    return src.received && (now - src.stamp).seconds() <= base_cmd_timeout_s_;
  }

  void step_base(bool any_anchored, const Eigen::Isometry3d *T_base_marker)
  {
    const rclcpp::Time now = this->now();
    const BaseSource &sm = sources_[kSpaceMouse];
    const BaseSource &lg = sources_[kLogitech];
    const bool sm_fresh = fresh(sm, now);
    if (sm_fresh && !is_zero(sm.twist)) {
      sm_last_active_ = now;
      sm_ever_active_ = true;
    }
    const bool sm_recent =
      sm_ever_active_ && (now - sm_last_active_).seconds() < spacemouse_idle_s_;

    const geometry_msgs::msg::Twist *in = nullptr;
    if (sm_fresh && sm_recent) in = &sm.twist;       // SpaceMouse driving (or just stopped)
    else if (fresh(lg, now)) in = &lg.twist;          // SpaceMouse idle >= spacemouse_idle_s
    else if (sm_fresh) in = &sm.twist;                // idle SpaceMouse zeros

    if (in == nullptr) {
      if (base_output_active_) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());   // stop once, then silence
        base_output_active_ = false;
        last_out_ = geometry_msgs::msg::Twist();
      }
      return;
    }

    geometry_msgs::msg::Twist out = *in;
    if (halted_) out = geometry_msgs::msg::Twist();   // D4: base forced to zero
    if (any_anchored) scale_twist(out, anchored_max_linear_, anchored_max_angular_);
    if (any_anchored && T_base_marker) clamp_base(out, *T_base_marker);
    cmd_vel_pub_->publish(out);
    last_out_ = out;
    base_output_active_ = true;
  }

  // Base motion over h seconds of twist t, in the current base frame (planar).
  static Eigen::Isometry3d base_step(const geometry_msgs::msg::Twist &t, double h)
  {
    Eigen::Isometry3d D = Eigen::Isometry3d::Identity();
    D.translation() = Eigen::Vector3d(t.linear.x * h, t.linear.y * h, 0.0);
    D.linear() = Eigen::AngleAxisd(t.angular.z * h, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return D;
  }

  static double box_violation(const Arm &arm, const Eigen::Vector3d &p)
  {
    const Eigen::Vector3d below = (arm.box_min - p).cwiseMax(0.0);
    const Eigen::Vector3d above = (p - arm.box_max).cwiseMax(0.0);
    return (below + above).norm();
  }

  bool achieved_fresh(const Arm &arm) const
  {
    return arm.achieved_valid &&
           (this->now() - arm.achieved_stamp).seconds() <= achieved_max_age_s_;
  }

  // Goal of an anchored arm in the base frame after the base moves by twist
  // t for h seconds.
  Eigen::Isometry3d goal_after(const Arm &arm, const Eigen::Isometry3d &T_base_marker,
                               const geometry_msgs::msg::Twist &t, double h) const
  {
    return base_step(t, h).inverse() * T_base_marker * arm.goal;
  }

  void clamp_base(geometry_msgs::msg::Twist &out, const Eigen::Isometry3d &T_bm)
  {
    if (is_zero(out)) return;
    const auto anchored = [](const Arm &a) { return a.valid && a.anchored && !a.held; };

    // ── Predictive: largest s in [0, 1] keeping every goal in its box ──
    auto ok = [&](double s) {
      geometry_msgs::msg::Twist ts = out;
      scale_twist_by(ts, s);
      for (const Arm &arm : arms_) {
        if (!anchored(arm)) continue;
        const double now_v = box_violation(arm, (T_bm * arm.goal).translation());
        const double new_v =
          box_violation(arm, goal_after(arm, T_bm, ts, clamp_horizon_s_).translation());
        if (new_v > now_v + 1e-6) return false;
      }
      return true;
    };
    if (!ok(1.0)) {
      double lo = 0.0, hi = 1.0;
      for (int i = 0; i < 12; ++i) {
        const double mid = 0.5 * (lo + hi);
        (ok(mid) ? lo : hi) = mid;
      }
      scale_twist_by(out, lo);
      if (lo < 0.99) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Base clamp: twist scaled to %.0f %% (anchored goal at its reach box)", lo * 100.0);
      if (is_zero(out) || lo == 0.0) return;
    }

    // ── Reactive backstop: arm lagging its goal while the base moves ──
    bool any_tripped = false, all_clear = true;
    for (const Arm &arm : arms_) {
      if (!anchored(arm) || !achieved_fresh(arm)) continue;
      const Eigen::Isometry3d gb = T_bm * arm.goal;
      const double dp = (gb.translation() - arm.achieved.translation()).norm();
      const double da = Eigen::AngleAxisd(gb.linear() * arm.achieved.linear().transpose()).angle();
      if (dp > trip_m_ || da > trip_rad_) any_tripped = true;
      if (dp > clear_m_ || da > clear_rad_) all_clear = false;
    }
    if (!reactive_tripped_ && any_tripped) {
      reactive_tripped_ = true;
      RCLCPP_WARN(this->get_logger(),
        "Base clamp: arm lags its anchored goal (> %.0f mm / %.1f deg) -- only "
        "error-reducing base motion until it catches up", trip_m_ * 1000.0, trip_rad_ * 180.0 / M_PI);
    } else if (reactive_tripped_ && all_clear) {
      reactive_tripped_ = false;
      RCLCPP_INFO(this->get_logger(), "Base clamp: arms caught up, base released");
    }
    if (!reactive_tripped_) return;
    for (const Arm &arm : arms_) {
      if (!anchored(arm) || !achieved_fresh(arm)) continue;
      const Eigen::Vector3d a = arm.achieved.translation();
      const double before = ((T_bm * arm.goal).translation() - a).norm();
      // one tick ahead: the direction must close the gap (a longer horizon
      // would overshoot the arm and reject the twist that catches it up)
      const double after = (goal_after(arm, T_bm, out, tick_dt_).translation() - a).norm();
      if (after > before - 1e-6 && before > clear_m_) {
        out = geometry_msgs::msg::Twist();
        return;
      }
    }
  }

  static void scale_twist_by(geometry_msgs::msg::Twist &t, double s)
  {
    t.linear.x *= s;
    t.linear.y *= s;
    t.linear.z *= s;
    t.angular.x *= s;
    t.angular.y *= s;
    t.angular.z *= s;
  }

  // Scale the WHOLE twist by one factor so |v_xy| <= max_lin, |w_z| <= max_ang.
  static void scale_twist(geometry_msgs::msg::Twist &t, double max_lin, double max_ang)
  {
    const double v = std::hypot(t.linear.x, t.linear.y);
    const double w = std::abs(t.angular.z);
    double s = 1.0;
    if (v > max_lin) s = std::min(s, max_lin / v);
    if (w > max_ang) s = std::min(s, max_ang / w);
    scale_twist_by(t, s);
  }

  void publish_goal(Arm &arm, const Eigen::Isometry3d &goal_base)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = this->now();
    out.header.frame_id = "map";
    out.pose = eigen_to_pose(goal_base);
    arm.goal_pub->publish(out);
  }

  std::string base_frame_;
  std::string marker_frame_;
  double marker_max_age_s_ {0.3};
  double external_hold_timeout_s_ {1.0};
  std::array<Arm, 2> arms_;

  double base_cmd_timeout_s_ {0.3};
  double spacemouse_idle_s_ {0.3};
  double anchored_max_linear_ {0.10};
  double anchored_max_angular_ {0.20};
  std::array<BaseSource, 2> sources_;
  rclcpp::Time sm_last_active_;
  bool sm_ever_active_ {false};
  bool base_output_active_ {false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist last_out_;

  // ── Base clamp (D5) ──
  double clamp_horizon_s_ {0.3};
  double trip_m_ {0.02};
  double trip_rad_ {3.0 * M_PI / 180.0};
  double clear_m_ {0.01};
  double clear_rad_ {1.5 * M_PI / 180.0};
  double achieved_max_age_s_ {0.5};
  bool reactive_tripped_ {false};
  double tick_dt_ {0.01};   // last /cmd_vel sent (EKF jump prediction)

  // ── Lift (D7) + head ──
  std::string mode_ {"ARM"};
  double lift_max_speed_ {0.10};
  double pending_lift_sm_ {0.0};   // SpaceMouse rate input, this tick only
  double pending_lift_lg_ {0.0};   // Logitech targets, ramped out
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> lift_subs_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_sm_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_lg_sub_;
  rclcpp::Time head_sm_last_;
  bool head_sm_seen_ {false};

  // ── Marker health (D4) ──
  bool marker_health_ {false};
  bool halted_ {false};
  double detection_timeout_s_ {2.0};
  double resume_healthy_s_ {1.0};
  double jump_max_m_ {0.05};
  double jump_max_rad_ {3.0 * M_PI / 180.0};
  double warn_min_rate_hz_ {6.4};
  double warn_max_gap_s_ {0.6};
  std::deque<rclcpp::Time> detections_;   // receive times, last 2 s
  rclcpp::Time last_detection_;
  bool have_detection_ {false};
  rclcpp::Time healthy_since_;
  bool healthy_since_valid_ {false};
  rclcpp::Time tf_stamp_;
  Eigen::Isometry3d prev_tf_ {Eigen::Isometry3d::Identity()};
  rclcpp::Time prev_tf_stamp_;
  bool have_prev_tf_ {false};
  Eigen::Isometry3d last_good_tf_ {Eigen::Isometry3d::Identity()};
  bool have_last_good_tf_ {false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr halted_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detection_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopCore>());
  rclcpp::shutdown();
  return 0;
}
