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
#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_stamped.hpp"
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
    if (!any_anchored) return;

    Eigen::Isometry3d T_base_marker;
    if (!lookup_base_from_marker(T_base_marker)) return;
    for (Arm &arm : arms_) {
      if (arm.valid && arm.anchored && !arm.held) publish_goal(arm, T_base_marker * arm.goal);
    }
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
