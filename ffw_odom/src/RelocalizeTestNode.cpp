#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "rf2o_laser_odometry/scan_to_map_icp.hpp"
#include "rf2o_laser_odometry/geometric_relocalizer.hpp"

using icp2d::Point2D;
using icp2d::Pose2D;
using icp2d::LineSegment;
using icp2d::ScanToMapICP;
using icp2d::GeometricRelocalizer;
using icp2d::ICPConfig;

// Re-use map loader helper
std::vector<Point2D> loadMapFromCsv(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open map file: " + path);
  }

  std::vector<Point2D> points;
  std::string line;
  size_t line_no = 0;
  bool header_skipped = false;
  while (std::getline(file, line)) {
    ++line_no;
    size_t start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
      continue; // blank line
    if (line[start] == '#')
      continue; // comment

    for (char &c : line)
      if (c == ',')
        c = ' '; // allow commas too
    std::istringstream iss(line);

    double wall_id, x, y;
    if (!(iss >> wall_id >> x >> y)) {
      if (!header_skipped) {
        header_skipped = true;
        continue;
      }
      throw std::runtime_error("Malformed map file line " +
                               std::to_string(line_no) + ": '" + line + "'");
    }
    header_skipped = true;
    points.push_back({x, y});
  }
  return points;
}

class RelocalizeTestNode : public rclcpp::Node {
public:
  RelocalizeTestNode() : Node("relocalize_test_node") {
    // Parameters
    declare_parameter<std::string>("map_file", "/home/lys/robotis_ws/src/ai_worker/ffw_odom/config/all_walls_downsampled_rotated.txt");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("scan_topic", "/scan");

    map_file_ = get_parameter("map_file").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();

    RCLCPP_INFO(get_logger(), "Loading map from '%s'...", map_file_.c_str());
    std::vector<Point2D> map_points = loadMapFromCsv(map_file_);
    RCLCPP_INFO(get_logger(), "Loaded %zu points from CSV map.", map_points.size());

    // Initialize modules
    ICPConfig cfg;
    cfg.verbose = false;
    matcher_ = std::make_unique<ScanToMapICP>(map_points, cfg);
    relocalizer_ = std::make_unique<GeometricRelocalizer>(map_points);

    // Subscriptions
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RelocalizeTestNode::scanCallback, this, std::placeholders::_1));

    // Service to trigger relocalization
    reloc_srv_ = create_service<std_srvs::srv::Trigger>(
        "relocalize",
        std::bind(&RelocalizeTestNode::triggerRelocalize, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Publishers for debugging
    map_lines_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/map_lines", 10);
    scan_lines_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/scan_lines", 10);
    scan_transformed_reloc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/scan_transformed_reloc", 10);
    scan_transformed_icp_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/scan_transformed_icp", 10);

    RCLCPP_INFO(get_logger(), "Relocalization test node initialized and ready.");
  }

private:
  void triggerRelocalize(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    need_reloc_ = true;
    response->success = true;
    response->message = "Global relocalization triggered.";
    RCLCPP_INFO(get_logger(), "Global relocalization triggered via service.");
  }

  sensor_msgs::msg::PointCloud2 pointsToCloudMsg(const std::vector<Point2D> &pts, const std::string &frame_id, rclcpp::Time stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = pts.size();
    msg.is_dense = true;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pts.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    for (const auto &p : pts) {
      *iter_x = static_cast<float>(p.x);
      *iter_y = static_cast<float>(p.y);
      *iter_z = 0.0f;
      ++iter_x; ++iter_y; ++iter_z;
    }
    return msg;
  }

  std::vector<Point2D> sampleLineSegments(const std::vector<LineSegment> &segs) {
    std::vector<Point2D> sampled;
    for (const auto &seg : segs) {
      double dx = seg.p2.x - seg.p1.x;
      double dy = seg.p2.y - seg.p1.y;
      double len = std::sqrt(dx*dx + dy*dy);
      if (len < 1e-4) continue;
      
      int num_samples = std::max(5, static_cast<int>(len / 0.02)); // 2cm spacing
      for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        sampled.push_back(Point2D{seg.p1.x + t * dx, seg.p1.y + t * dy});
      }
    }
    return sampled;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<Point2D> scan_points;
    scan_points.reserve(msg->ranges.size());
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
        continue;
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      scan_points.push_back({r * std::cos(angle), r * std::sin(angle)});
    }

    if (scan_points.size() < 10) return;

    // --- STAGE 2: Extract map segments ---
    std::vector<LineSegment> map_segs = relocalizer_->extractSegmentsFromMap();
    std::vector<Point2D> map_line_points = sampleLineSegments(map_segs);

    Pose2D active_pose;
    Pose2D guess_pose;

    if (need_reloc_) {
      RCLCPP_INFO(get_logger(), "Running global geometric relocalization...");
      active_pose = relocalizer_->relocalize(scan_points, *matcher_);
      
      guess_pose = active_pose;
      auto scan_segs_raw = relocalizer_->extractSegmentsFromScan(scan_points);
      auto map_segs_raw = relocalizer_->extractSegmentsFromMap();
      const int num_bins = 360;
      std::vector<double> histogram(num_bins, 0.0);
      for (const auto &ss : scan_segs_raw) {
        for (const auto &ms : map_segs_raw) {
          double theta1 = ms.phi - ss.phi;
          while (theta1 < -M_PI) theta1 += 2.0 * M_PI;
          while (theta1 > M_PI)  theta1 -= 2.0 * M_PI;

          double theta2 = theta1 + M_PI;
          if (theta2 > M_PI) theta2 -= 2.0 * M_PI;

          double weight = std::min(ss.length, ms.length);

          double deg1 = (theta1 + M_PI) * 180.0 / M_PI;
          int bin1 = std::clamp(static_cast<int>(std::round(deg1)), 0, num_bins - 1);
          histogram[bin1] += weight;

          double deg2 = (theta2 + M_PI) * 180.0 / M_PI;
          int bin2 = std::clamp(static_cast<int>(std::round(deg2)), 0, num_bins - 1);
          histogram[bin2] += weight;
        }
      }

      std::vector<double> smoothed(num_bins, 0.0);
      for (int i = 0; i < num_bins; ++i) {
        int prev = (i - 1 + num_bins) % num_bins;
        int next = (i + 1) % num_bins;
        smoothed[i] = 0.25 * histogram[prev] + 0.5 * histogram[i] + 0.25 * histogram[next];
      }

      double max_val = 0.0;
      for (double val : smoothed) max_val = std::max(max_val, val);
      std::vector<double> hypotheses;
      if (max_val > 0.01) {
        for (int bin = 0; bin < num_bins; ++bin) {
          if (smoothed[bin] >= 0.8 * max_val) {
            int prev_bin = (bin - 1 + num_bins) % num_bins;
            int next_bin = (bin + 1) % num_bins;
            if (smoothed[bin] >= smoothed[prev_bin] && smoothed[bin] >= smoothed[next_bin]) {
              double y_prev = smoothed[prev_bin];
              double y_curr = smoothed[bin];
              double y_next = smoothed[next_bin];

              double denom = 2.0 * (y_prev - 2.0 * y_curr + y_next);
              double offset = 0.0;
              if (std::abs(denom) > 1e-5) {
                offset = (y_prev - y_next) / denom;
              }
              offset = std::clamp(offset, -0.5, 0.5);

              double peak_deg = static_cast<double>(bin) + offset;
              double theta = (peak_deg / 180.0 * M_PI) - M_PI;
              while (theta < -M_PI) theta += 2.0 * M_PI;
              while (theta > M_PI)  theta -= 2.0 * M_PI;

              hypotheses.push_back(theta);
            }
          }
        }
      }

      std::vector<Pose2D> candidates;
      for (double theta : hypotheses) {
        struct MiniMatch { double m_phi, m_rho, s_rho; };
        std::vector<MiniMatch> matches;
        for (const auto &ss : scan_segs_raw) {
          double ss_phi_rot = ss.phi + theta;
          while (ss_phi_rot < -M_PI) ss_phi_rot += 2.0 * M_PI;
          while (ss_phi_rot > M_PI)  ss_phi_rot -= 2.0 * M_PI;
          for (const auto &ms : map_segs_raw) {
            double d_phi = std::abs(ms.phi - ss_phi_rot);
            if (d_phi > M_PI) d_phi = 2.0 * M_PI - d_phi;
            if (d_phi < 0.26) {
              matches.push_back({ms.phi, ms.rho, ss.rho});
            } else if (std::abs(d_phi - M_PI) < 0.26) {
              matches.push_back({ms.phi, ms.rho, -ss.rho});
            }
          }
        }
        if (matches.size() >= 2) {
          Eigen::MatrixXd A(matches.size(), 2);
          Eigen::VectorXd B(matches.size());
          for (size_t i = 0; i < matches.size(); ++i) {
            A(i, 0) = std::cos(matches[i].m_phi);
            A(i, 1) = std::sin(matches[i].m_phi);
            B(i) = matches[i].m_rho - matches[i].s_rho;
          }
          Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
          Eigen::Vector2d X_Y = svd.solve(B);
          candidates.push_back(Pose2D{X_Y(0), X_Y(1), theta});
        }
      }
      if (candidates.size() > 0) {
        int best_cand_inliers = -1;
        for (const auto &cand : candidates) {
          int inliers = 0;
          for (const auto &sp : scan_points) {
            Point2D tp = cand.apply(sp);
            for (const auto &ms : map_segs_raw) {
              double d = std::abs(tp.x * std::cos(ms.phi) + tp.y * std::sin(ms.phi) - ms.rho);
              if (d < 0.3) { inliers++; break; }
            }
          }
          if (inliers > best_cand_inliers) {
            best_cand_inliers = inliers;
            guess_pose = cand;
          }
        }
      }
      
      current_pose_ = active_pose;
      need_reloc_ = false;
      RCLCPP_INFO(get_logger(), "Global relocalization solved! Locking pose: [%.2f, %.2f, %.1f deg]",
                  current_pose_.x, current_pose_.y, current_pose_.theta * 180.0 / M_PI);
    } else {
      guess_pose = current_pose_;
      icp2d::ICPResult res = matcher_->align(scan_points, current_pose_);
      active_pose = res.corrected_pose;
      current_pose_ = active_pose;
    }



    // Transform extracted scan segments to map frame using active_pose
    std::vector<LineSegment> scan_segs = relocalizer_->extractSegmentsFromScan(scan_points);
    for (auto &seg : scan_segs) {
      seg.p1 = active_pose.apply(seg.p1);
      seg.p2 = active_pose.apply(seg.p2);
    }
    std::vector<Point2D> scan_line_points = sampleLineSegments(scan_segs);

    // Apply translations to raw scan points for overlays
    std::vector<Point2D> scan_transformed_reloc;
    for (const auto &sp : scan_points) {
      scan_transformed_reloc.push_back(guess_pose.apply(sp));
    }

    std::vector<Point2D> scan_transformed_icp;
    for (const auto &sp : scan_points) {
      scan_transformed_icp.push_back(active_pose.apply(sp));
    }

    // Publish all visual debug topics in map frame!
    auto stamp = msg->header.stamp;
    map_lines_pub_->publish(pointsToCloudMsg(map_line_points, map_frame_, stamp));
    scan_lines_pub_->publish(pointsToCloudMsg(scan_line_points, map_frame_, stamp));
    scan_transformed_reloc_pub_->publish(pointsToCloudMsg(scan_transformed_reloc, map_frame_, stamp));
    scan_transformed_icp_pub_->publish(pointsToCloudMsg(scan_transformed_icp, map_frame_, stamp));

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Published debug clouds | Scan Segments: %zu | Map Segments: %zu | pose: [%.2f, %.2f, %.1f deg]",
        scan_segs.size(), map_segs.size(), active_pose.x, active_pose.y, active_pose.theta * 180.0 / M_PI);
  }

  std::string map_file_, map_frame_, scan_topic_;
  std::shared_ptr<ScanToMapICP> matcher_;
  std::shared_ptr<GeometricRelocalizer> relocalizer_;
  Pose2D current_pose_{0.0, 0.0, 0.0};
  bool need_reloc_ = true;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reloc_srv_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_lines_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_lines_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_transformed_reloc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_transformed_icp_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelocalizeTestNode>());
  rclcpp::shutdown();
  return 0;
}
