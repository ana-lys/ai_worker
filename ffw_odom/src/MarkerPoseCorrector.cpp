// MarkerPoseCorrector.cpp
//
// Converts /oakd/marker_board_pose (PoseStamped, T_baselink_marker, ~4-5 Hz
// AprilTag detections) into an absolute base_link-in-marker_frame pose
// measurement for robot_localization's ekf_node to fuse with continuous
// wheel/encoder odometry (see marker_ekf.yaml / marker_ekf.launch.py).
//
// Two things happen to the raw detection:
//   1. A fixed yaw correction about the marker's own local Z axis (the raw
//      AprilTag axis convention doesn't match ROS front/left/up) -- same
//      correction previously done by the (now removed) raw TF broadcaster.
//   2. An inversion: T_baselink_markerFrame -> T_markerFrame_baselink, since
//      the EKF's pose input wants "base_link's pose expressed in the world
//      (marker_frame) frame", not "the marker's pose expressed in base_link".
//
// This node does NOT broadcast any TF itself -- the ekf_node it feeds does
// that (publish_tf: true in marker_ekf.yaml), producing a smooth, high-rate
// marker_frame <-> base_link transform instead of a jerky 4-5 Hz relay.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

class MarkerPoseCorrectorNode : public rclcpp::Node {
public:
  MarkerPoseCorrectorNode() : Node("marker_pose_corrector") {
    source_frame_ = declare_parameter<std::string>("source_frame", "base_link");
    child_frame_ = declare_parameter<std::string>("child_frame", "marker_frame");
    yaw_offset_ = declare_parameter<double>("board_yaw_offset_rad", M_PI / 2.0);
    input_topic_ = declare_parameter<std::string>("input_topic", "/oakd/marker_board_pose");
    output_topic_ = declare_parameter<std::string>("output_topic", "/oakd/marker_frame_baselink_pose");
    position_stddev_m_ = declare_parameter<double>("position_stddev_m", 0.02);
    yaw_stddev_rad_ = declare_parameter<double>("yaw_stddev_rad", 0.05);
    orientation_stddev_rad_ = declare_parameter<double>("orientation_stddev_rad", 0.1);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, rclcpp::QoS(5));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&MarkerPoseCorrectorNode::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "marker_pose_corrector: %s -> %s, yaw_offset=%.3f rad, sub=%s pub=%s",
      source_frame_.c_str(), child_frame_.c_str(), yaw_offset_,
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg->header.frame_id != source_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "marker_board_pose frame_id '%s' != configured source_frame '%s' -- "
        "correcting anyway",
        msg->header.frame_id.c_str(), source_frame_.c_str());
    }

    // Same intrinsic-Z correction as the old raw broadcaster:
    // q_out = q_in * q_offset (rotates about the marker's own Z axis).
    tf2::Quaternion q_in(msg->pose.orientation.x, msg->pose.orientation.y,
                        msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Quaternion q_offset;
    q_offset.setRPY(0.0, 0.0, yaw_offset_);
    tf2::Quaternion q_out = (q_in * q_offset).normalized();

    // T_baselink_markerFrame (marker frame, as seen from base_link).
    tf2::Transform t_bl_marker(
      q_out,
      tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

    // The EKF wants "base_link's pose in marker_frame" -- the inverse.
    tf2::Transform t_marker_bl = t_bl_marker.inverse();
    const tf2::Vector3 &p = t_marker_bl.getOrigin();
    const tf2::Quaternion q = t_marker_bl.getRotation();

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = child_frame_;
    out.pose.pose.position.x = p.x();
    out.pose.pose.position.y = p.y();
    out.pose.pose.position.z = p.z();
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.pose.orientation.w = q.w();

    // Diagonal covariance, row-major 6x6 (x,y,z,roll,pitch,yaw). two_d_mode
    // in marker_ekf.yaml only consumes x,y,yaw so roll/pitch/z values here
    // are unused in practice but still filled with a sensible number.
    std::fill(out.pose.covariance.begin(), out.pose.covariance.end(), 0.0);
    const double pos_var = position_stddev_m_ * position_stddev_m_;
    const double ang_var = orientation_stddev_rad_ * orientation_stddev_rad_;
    const double yaw_var = yaw_stddev_rad_ * yaw_stddev_rad_;
    out.pose.covariance[0] = pos_var;   // x
    out.pose.covariance[7] = pos_var;   // y
    out.pose.covariance[14] = pos_var;  // z
    out.pose.covariance[21] = ang_var;  // roll
    out.pose.covariance[28] = ang_var;  // pitch
    out.pose.covariance[35] = yaw_var;  // yaw

    pose_pub_->publish(out);
  }

  std::string source_frame_, child_frame_, input_topic_, output_topic_;
  double yaw_offset_, position_stddev_m_, yaw_stddev_rad_, orientation_stddev_rad_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPoseCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}
