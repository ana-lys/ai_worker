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
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
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
    base_cmd_timeout_s_ = this->declare_parameter("base_cmd_timeout_s", 0.3);
    spacemouse_idle_s_ = this->declare_parameter("spacemouse_idle_s", 0.3);
    anchored_max_linear_ = this->declare_parameter("anchored_max_linear_vel", 0.10);
    anchored_max_angular_ = this->declare_parameter("anchored_max_angular_vel", 0.20);

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

    const std::array<std::string, 2> names {"left", "right"};
    for (size_t i = 0; i < arms_.size(); ++i) {
      Arm &arm = arms_[i];
      arm.name = names[i];
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
    if (hold_active(arm)) return;
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
    return true;
  }

  void tick()
  {
    bool any_anchored = false;
    for (Arm &arm : arms_) {
      if (arm.valid && arm.anchored && !hold_active(arm)) any_anchored = true;
    }
    Eigen::Isometry3d T_base_marker;
    if (any_anchored && lookup_base_from_marker(T_base_marker)) {
      for (Arm &arm : arms_) {
        if (arm.valid && arm.anchored && !arm.held) publish_goal(arm, T_base_marker * arm.goal);
      }
    }
    step_base(any_anchored);
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

  void step_base(bool any_anchored)
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
      }
      return;
    }

    geometry_msgs::msg::Twist out = *in;
    if (any_anchored) scale_twist(out, anchored_max_linear_, anchored_max_angular_);
    cmd_vel_pub_->publish(out);
    base_output_active_ = true;
  }

  // Scale the WHOLE twist by one factor so |v_xy| <= max_lin, |w_z| <= max_ang.
  static void scale_twist(geometry_msgs::msg::Twist &t, double max_lin, double max_ang)
  {
    const double v = std::hypot(t.linear.x, t.linear.y);
    const double w = std::abs(t.angular.z);
    double s = 1.0;
    if (v > max_lin) s = std::min(s, max_lin / v);
    if (w > max_ang) s = std::min(s, max_ang / w);
    t.linear.x *= s;
    t.linear.y *= s;
    t.linear.z *= s;
    t.angular.x *= s;
    t.angular.y *= s;
    t.angular.z *= s;
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
