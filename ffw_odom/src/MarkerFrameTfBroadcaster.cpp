// MarkerFrameTfBroadcaster.cpp
//
// Subscribes to /marker_ekf_odom (nav_msgs/Odometry, header.frame_id=
// "marker_frame", child_frame_id="base_link" -- base_link's smoothed pose
// in marker_frame, published by the marker EKF with publish_tf disabled)
// and broadcasts the INVERSE as TF: base_link -> marker_frame.
//
// Why the inversion: robot_localization's ekf_node always publishes its own
// TF edge as world_frame -> base_link_frame (base_link as the CHILD). But
// base_link already has a real parent from the existing odom/map stack --
// a second, independent parent for the same child splits the TF tree into
// two disconnected pieces ("Tf has two or more unconnected trees"), which
// broke scan_to_map_icp's odom<->base_link time-sync lookups. Publishing
// base_link -> marker_frame instead makes marker_frame a LEAF hanging off
// base_link, which cannot conflict with anything else in the tree -- and
// TF lookups are direction-agnostic, so consumers (joy_hand.cpp, the CLI)
// need no changes regardless of which direction actually gets broadcast.

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

class MarkerFrameTfBroadcasterNode : public rclcpp::Node {
public:
  MarkerFrameTfBroadcasterNode() : Node("marker_frame_tf_broadcaster") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/marker_ekf_odom");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&MarkerFrameTfBroadcasterNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "marker_frame_tf_broadcaster: relaying %s as <child_frame_id> -> "
      "<header.frame_id> TF, inverted", input_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // T_markerFrame_baselink (base_link's pose in marker_frame), as filtered
    // by the marker EKF.
    tf2::Transform t_marker_bl(
      tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w),
      tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y,
                   msg->pose.pose.position.z));
    // T_baselink_markerFrame -- the leaf-direction edge we actually broadcast.
    tf2::Transform t_bl_marker = t_marker_bl.inverse();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = msg->child_frame_id;   // "base_link" -- TF parent
    t.child_frame_id = msg->header.frame_id;   // "marker_frame" -- TF child (leaf)
    const tf2::Vector3 &p = t_bl_marker.getOrigin();
    const tf2::Quaternion q = t_bl_marker.getRotation();
    t.transform.translation.x = p.x();
    t.transform.translation.y = p.y();
    t.transform.translation.z = p.z();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  std::string input_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerFrameTfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
