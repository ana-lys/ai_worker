// joy_hand_marker: SpaceMouse EE teleop with goals anchored in marker_frame
// (the AprilTag board frame from the marker EKF). Moving the base keeps the
// EE goals fixed in marker_frame; SpaceMouse deltas act in marker axes.
#include "ffw_spacemouse/ee_teleop_base.hpp"

class MarkerFrameTeleop : public EeTeleopBase
{
public:
  MarkerFrameTeleop() : EeTeleopBase("marker_frame")
  {
    base_frame_ = this->declare_parameter("base_frame", "base_link");
    // marker_frame TF older than this = EKF dead: hold (MARKER_TELEOP_PLAN D4).
    marker_max_age_s_ = this->declare_parameter("marker_max_age_s", 0.3);
  }

protected:
  bool lookup_base_from_native(Eigen::Isometry3d &T_base_native) override
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(base_frame_, ee_goal_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s -> %s unavailable (%s) -- holding", ee_goal_frame_.c_str(),
        base_frame_.c_str(), ex.what());
      return false;
    }
    const double age = (this->now() - rclcpp::Time(t.header.stamp)).seconds();
    if (age > marker_max_age_s_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s -> %s is %.2f s old (> %.2f s) -- holding", ee_goal_frame_.c_str(),
        base_frame_.c_str(), age, marker_max_age_s_);
      return false;
    }
    T_base_native = transform_to_eigen(t);
    return true;
  }

private:
  std::string base_frame_;
  double marker_max_age_s_ {0.3};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerFrameTeleop>());
  rclcpp::shutdown();
  return 0;
}
