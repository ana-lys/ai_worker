#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <cmath>
#include <vector>
#include <string>

using std::placeholders::_1;

class LaserMergerNode : public rclcpp::Node
{
public:
  LaserMergerNode() : Node("ffw_laser_merger_node")
  {
    // Declare parameters for left bounding box (relative to left lidar, aligned with base_link)
    this->declare_parameter("left_min_x", -0.5);
    this->declare_parameter("left_max_x", 0.4);
    this->declare_parameter("left_min_y", -0.6);
    this->declare_parameter("left_max_y", 0.2);

    // Declare parameters for right bounding box (relative to right lidar, aligned with base_link)
    this->declare_parameter("right_min_x", -0.5);
    this->declare_parameter("right_max_x", 0.4);
    this->declare_parameter("right_min_y", -0.2);
    this->declare_parameter("right_max_y", 0.6);

    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("publish_rate", 30.0);

    // Get parameters
    left_min_x_ = this->get_parameter("left_min_x").as_double();
    left_max_x_ = this->get_parameter("left_max_x").as_double();
    left_min_y_ = this->get_parameter("left_min_y").as_double();
    left_max_y_ = this->get_parameter("left_max_y").as_double();

    right_min_x_ = this->get_parameter("right_min_x").as_double();
    right_max_x_ = this->get_parameter("right_max_x").as_double();
    right_min_y_ = this->get_parameter("right_min_y").as_double();
    right_max_y_ = this->get_parameter("right_max_y").as_double();

    target_frame_ = this->get_parameter("target_frame").as_string();
    double pub_rate = this->get_parameter("publish_rate").as_double();

    // TF Setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    sub_left_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_left_raw", rclcpp::SensorDataQoS(), std::bind(&LaserMergerNode::left_scan_cb, this, _1));
    sub_right_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_right_raw", rclcpp::SensorDataQoS(), std::bind(&LaserMergerNode::right_scan_cb, this, _1));

    // Publishers
    pub_merged_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/scan_debug_markers", 10);

    // Timer
    auto period = std::chrono::duration<double>(1.0 / pub_rate);
    timer_ = this->create_wall_timer(period, std::bind(&LaserMergerNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "FFW Laser Merger Node Initialized (Relative Base Link Filtering)");
  }

private:
  void left_scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    left_scan_ = msg;
  }

  void right_scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    right_scan_ = msg;
  }

  void timer_cb()
  {
    if (!left_scan_ && !right_scan_) return;

    // Use header from whatever scan we have, default to left if both exist
    auto base_scan = left_scan_ ? left_scan_ : right_scan_;

    sensor_msgs::msg::LaserScan merged_scan;
    merged_scan.header.stamp = base_scan->header.stamp;
    merged_scan.header.frame_id = target_frame_;
    
    // Setup standard 360 degree scan params
    merged_scan.angle_min = -M_PI;
    merged_scan.angle_max = M_PI;
    merged_scan.angle_increment = base_scan->angle_increment;
    merged_scan.time_increment = base_scan->time_increment;
    merged_scan.scan_time = base_scan->scan_time;
    merged_scan.range_min = base_scan->range_min;
    merged_scan.range_max = base_scan->range_max;

    int num_rays = std::ceil((merged_scan.angle_max - merged_scan.angle_min) / merged_scan.angle_increment);
    merged_scan.ranges.assign(num_rays, std::numeric_limits<float>::infinity());
    merged_scan.intensities.assign(num_rays, 0.0);

    // Process Left Scan
    if (left_scan_) {
      process_scan(left_scan_, merged_scan, left_min_x_, left_max_x_, left_min_y_, left_max_y_);
    }

    // Process Right Scan
    if (right_scan_) {
      process_scan(right_scan_, merged_scan, right_min_x_, right_max_x_, right_min_y_, right_max_y_);
    }

    pub_merged_->publish(merged_scan);
    publish_debug_markers();
  }

  void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr& scan, 
                    sensor_msgs::msg::LaserScan& merged_scan,
                    double min_x, double max_x, double min_y, double max_y)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(target_frame_, scan->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                     "Could not transform %s to %s: %s", 
                                     scan->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float r = scan->ranges[i];
      if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) {
        continue;
      }

      double angle = scan->angle_min + i * scan->angle_increment;
      double local_x = r * std::cos(angle);
      double local_y = r * std::sin(angle);

      // Transform to target frame (base_link) FIRST
      geometry_msgs::msg::PointStamped pt_in, pt_out;
      pt_in.header = scan->header;
      pt_in.point.x = local_x;
      pt_in.point.y = local_y;
      pt_in.point.z = 0.0;

      tf2::doTransform(pt_in, pt_out, transform);

      // Calculate the point's position relative to the lidar's origin, but in the target_frame's orientation
      double rel_x = pt_out.point.x - transform.transform.translation.x;
      double rel_y = pt_out.point.y - transform.transform.translation.y;

      // Apply bounding box filter
      if (rel_x >= min_x && rel_x <= max_x && rel_y >= min_y && rel_y <= max_y) {
        continue; // Drop the point because it hit the robot body
      }

      // Re-project into the merged scan
      double out_angle = std::atan2(pt_out.point.y, pt_out.point.x);
      double out_range = std::hypot(pt_out.point.x, pt_out.point.y);

      if (out_range < merged_scan.range_min || out_range > merged_scan.range_max) {
        continue;
      }

      // Normalize angle to [angle_min, angle_max] if needed, though atan2 is [-pi, pi]
      int index = std::round((out_angle - merged_scan.angle_min) / merged_scan.angle_increment);
      if (index >= 0 && index < (int)merged_scan.ranges.size()) {
        if (out_range < merged_scan.ranges[index]) {
          merged_scan.ranges[index] = out_range;
          if (scan->intensities.size() > i) {
            merged_scan.intensities[index] = scan->intensities[i];
          }
        }
      }
    }
  }

  void publish_debug_markers()
  {
    visualization_msgs::msg::MarkerArray markers;

    if (left_scan_) {
      markers.markers.push_back(create_rect_marker(target_frame_, left_scan_->header.frame_id, 0, 
                                                   left_min_x_, left_max_x_, left_min_y_, left_max_y_));
    }
    if (right_scan_) {
      markers.markers.push_back(create_rect_marker(target_frame_, right_scan_->header.frame_id, 1, 
                                                   right_min_x_, right_max_x_, right_min_y_, right_max_y_));
    }

    pub_markers_->publish(markers);
  }

  visualization_msgs::msg::Marker create_rect_marker(const std::string& target_frame, const std::string& lidar_frame, int id, 
                                                     double min_x, double max_x, double min_y, double max_y)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = target_frame;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "laser_box_border";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02; // Line width

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // We need to offset the marker by the lidar's translation in base_link
    geometry_msgs::msg::TransformStamped transform;
    double t_x = 0.0;
    double t_y = 0.0;
    try {
      transform = tf_buffer_->lookupTransform(target_frame, lidar_frame, tf2::TimePointZero);
      t_x = transform.transform.translation.x;
      t_y = transform.transform.translation.y;
    } catch (const tf2::TransformException & ex) {
      // Just return empty marker if transform fails
      return marker;
    }

    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = t_x + min_x; p1.y = t_y + min_y; p1.z = 0;
    p2.x = t_x + max_x; p2.y = t_y + min_y; p2.z = 0;
    p3.x = t_x + max_x; p3.y = t_y + max_y; p3.z = 0;
    p4.x = t_x + min_x; p4.y = t_y + max_y; p4.z = 0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);
    marker.points.push_back(p1);

    return marker;
  }

  double left_min_x_, left_max_x_, left_min_y_, left_max_y_;
  double right_min_x_, right_max_x_, right_min_y_, right_max_y_;
  std::string target_frame_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_right_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_merged_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::SharedPtr left_scan_;
  sensor_msgs::msg::LaserScan::SharedPtr right_scan_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserMergerNode>());
  rclcpp::shutdown();
  return 0;
}
