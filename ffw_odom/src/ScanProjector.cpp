// ScanProjector.cpp
//
// Projects live LIDAR scans into the map frame using the EKF pose estimate
// and publishes the result as a PointCloud2 for visualisation, debugging, or
// downstream mapping.
//
// Subscribes:
//   /scan      — sensor_msgs/LaserScan   (raw lidar)
//   /ekf_odom  — nav_msgs/Odometry       (robot_localisation EKF output;
//                                         world_frame=map so the pose IS
//                                         the map→base_link transform)
//
// Publishes:
//   /map_scan  — sensor_msgs/PointCloud2  (projected scan in the map frame)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <optional>

namespace ffw_odom {

// -----------------------------------------------------------------------
// Node
// -----------------------------------------------------------------------
class ScanProjectorNode : public rclcpp::Node {
public:
  ScanProjectorNode() : Node("scan_projector") {
    // --- parameters ---
    scan_topic_     = declare_parameter("scan_topic", "/scan");
    ekf_odom_topic_ = declare_parameter("ekf_odom_topic", "/ekf_odom");
    output_topic_   = declare_parameter("output_topic", "/map_scan");
    base_frame_     = declare_parameter("base_frame", "base_link");
    map_frame_      = declare_parameter("map_frame", "map");

    // --- TF ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- subscribers ---
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10,
        std::bind(&ScanProjectorNode::scanCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        ekf_odom_topic_, 10,
        std::bind(&ScanProjectorNode::odomCallback, this, std::placeholders::_1));

    // --- publisher ---
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "ScanProjector ready: %s → %s in frame %s",
                scan_topic_.c_str(), output_topic_.c_str(), map_frame_.c_str());
  }

private:
  // -----------------------------------------------------------------------
  // Cache the latest EKF pose (map→base_link)
  // -----------------------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    latest_ekf_pose_ = msg->pose.pose;
  }

  // -----------------------------------------------------------------------
  // Project scan into map frame and publish
  // -----------------------------------------------------------------------
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // ---- 1.  Look up laser→base_link transform (cache once) ----
    if (!laser_to_base_.has_value()) {
      try {
        geometry_msgs::msg::TransformStamped tf =
            tf_buffer_->lookupTransform(
                base_frame_, msg->header.frame_id, msg->header.stamp,
                std::chrono::milliseconds(100));
        double theta = tf2::getYaw(tf.transform.rotation);
        LTB ltb;
        ltb.tx  = tf.transform.translation.x;
        ltb.ty  = tf.transform.translation.y;
        ltb.cos = std::cos(theta);
        ltb.sin = std::sin(theta);
        laser_to_base_ = ltb;
        RCLCPP_INFO(get_logger(), "Cached laser→base_link: [%.3f, %.3f, %.3f]",
                    ltb.tx, ltb.ty, theta);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cannot get laser→base_link TF: %s", ex.what());
        return;
      }
    }

    // ---- 2.  Get latest EKF pose (map→base_link) ----
    geometry_msgs::msg::Pose ekf_pose;
    {
      std::lock_guard<std::mutex> lock(ekf_mutex_);
      ekf_pose = latest_ekf_pose_;
    }
    // If we never received an EKF message yet, the default-constructed Pose
    // is {0,0,0, w=1} — that's the identity transform, which is acceptable
    // for the first frame (just projects the scan onto x/y in the map frame).

    // Precompute map→base_link rotation
    const auto &q = ekf_pose.orientation;
    double map_theta = tf2::getYaw(q);
    double c = std::cos(map_theta), s = std::sin(map_theta);
    double map_x = ekf_pose.position.x;
    double map_y = ekf_pose.position.y;

    // ---- 3.  Convert and project ----
    const auto &ltb = *laser_to_base_;

    // First pass: count valid points so we can size the cloud exactly.
    size_t valid = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max)
        ++valid;
    }

    if (valid == 0)
      return;

    // Build the point cloud
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = map_frame_;
    cloud->header.stamp    = msg->header.stamp;
    cloud->height  = 1;
    cloud->width   = valid;
    cloud->is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.reserve(valid);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
        continue;

      const double angle =
          msg->angle_min + static_cast<double>(i) * msg->angle_increment;

      // Point in laser frame
      double lx = r * std::cos(angle);
      double ly = r * std::sin(angle);

      // laser → base_link  (2D rotation + translation)
      double bx = ltb.cos * lx - ltb.sin * ly + ltb.tx;
      double by = ltb.sin * lx + ltb.cos * ly + ltb.ty;

      // base_link → map
      double mx = c * bx - s * by + map_x;
      double my = s * bx + c * by + map_y;

      *iter_x = static_cast<float>(mx);  ++iter_x;
      *iter_y = static_cast<float>(my);  ++iter_y;
      *iter_z = 0.0f;                    ++iter_z;
    }

    cloud_pub_->publish(std::move(cloud));
  }

  // --- parameters ---
  std::string scan_topic_;
  std::string ekf_odom_topic_;
  std::string output_topic_;
  std::string base_frame_;
  std::string map_frame_;

  // --- TF ---
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Cached laser→base_link transform — precomputed cos/sin so we never
  // call trig per point in the inner loop.
  struct LTB { double tx, ty, cos, sin; };
  std::optional<LTB> laser_to_base_;

  // --- EKF state (protected by mutex; written on /ekf_odom, read on /scan) ---
  std::mutex ekf_mutex_;
  geometry_msgs::msg::Pose latest_ekf_pose_;

  // --- ROS handles ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace ffw_odom

// -----------------------------------------------------------------------
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ffw_odom::ScanProjectorNode>());
  rclcpp::shutdown();
  return 0;
}
