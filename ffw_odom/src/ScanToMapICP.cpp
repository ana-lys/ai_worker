// ScanToMapICP.cpp
//
// ROS2 node that:
//   - Loads a static map (pre-filtered wall/line features) from a CSV file at
//     startup: one "x,y" pair per line, '#' comments allowed,
//     whitespace-tolerant.
//   - Subscribes to a single merged sensor_msgs/LaserScan (dual-LakiBeam1,
//     already merged upstream into one topic).
//   - Subscribes to nav_msgs/Odometry for the raw (drifting) odometry.
//   - Runs ScanToMapICP each scan, using the current odom + last known
//     map->odom offset as the ICP initial guess.
//   - On a converged/accepted match, updates the map->odom offset.
//   - Every scan (whether or not this one matched), publishes:
//       * a corrected nav_msgs/Odometry on ~/odom_corrected (frame: map_frame,
//         child frame: base_frame)
//       * a TF broadcast map_frame -> odom_frame
//     This keeps the existing odom_frame -> base_frame TF chain (from your
//     wheel odometry / rf2o node) completely untouched -- only the map->odom
//     offset is adjusted, which is the standard localization-correction
//     pattern.
//
// Threading note: uses the default SingleThreadedExecutor assumption (see
// main()) so the odom and scan callbacks never run concurrently; no extra
// locking is needed. If you switch to a MultiThreadedExecutor or callback
// groups, add a mutex around current_odom_pose_/current_odom_twist_.

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <mutex>
#include <condition_variable>

#include "rf2o_laser_odometry/scan_to_map_icp.hpp"
#include "rf2o_laser_odometry/geometric_relocalizer.hpp"

using icp2d::ICPConfig;
using icp2d::ICPResult;
using icp2d::Point2D;
using icp2d::Pose2D;
using icp2d::ScanToMapICP;
using icp2d::GeometricRelocalizer;

namespace {

// Loads map points from a whitespace-separated text file with columns
// "wall_id x y" (a header row is expected and skipped; wall_id is ignored,
// only x/y are kept). Blank lines and lines starting with '#' are skipped.
// Comma-separated files also work fine (commas are treated as whitespace).
// Throws std::runtime_error on I/O failure or a malformed data line.
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
      // First non-blank, non-comment line that fails to parse as three
      // numbers is treated as the "wall_id x y" header and skipped once.
      if (!header_skipped) {
        header_skipped = true;
        continue;
      }
      throw std::runtime_error("Malformed map file line " +
                               std::to_string(line_no) + ": '" + line + "'");
    }
    header_skipped = true;
    points.push_back({x, y}); // wall_id intentionally discarded
  }
  return points;
}

Pose2D poseFromOdomMsg(const nav_msgs::msg::Odometry &msg) {
  Pose2D p;
  p.x = msg.pose.pose.position.x;
  p.y = msg.pose.pose.position.y;
  p.theta = tf2::getYaw(msg.pose.pose.orientation);
  return p;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

} // namespace

class ScanToMapICPNode : public rclcpp::Node {
public:
  ScanToMapICPNode() : rclcpp::Node("scan_to_map_icp") {
    // --- parameters ---
    map_file_ = declare_parameter<std::string>("map_file", "");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    corrected_odom_topic_ = declare_parameter<std::string>(
        "corrected_odom_topic", "/icp_pose_raw");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    ICPConfig cfg;
    cfg.max_correspondence_dist =
        declare_parameter<double>("max_correspondence_dist", 0.15);
    cfg.huber_delta = declare_parameter<double>("huber_delta", 0.10);
    cfg.max_iterations = declare_parameter<int>("max_iterations", 50);
    cfg.min_inlier_ratio = declare_parameter<double>("min_inlier_ratio", 0.25);
    cfg.normal_k_neighbors = declare_parameter<int>("normal_k_neighbors", 8);
    cfg.translation_eps = declare_parameter<double>("translation_eps", 1e-4);
    cfg.rotation_eps = declare_parameter<double>("rotation_eps", 1e-5);
    cfg.verbose = declare_parameter<bool>("verbose", false);
    max_accepted_rms_ = declare_parameter<double>("max_accepted_rms", 0.15);
    fallback_hysteresis_ = declare_parameter<int>("fallback_hysteresis", 3);
    scan_to_scan_hysteresis_ = declare_parameter<int>("scan_to_scan_hysteresis", 2);
    bool start_active = declare_parameter<bool>("start_active", false);
    active_ = start_active;

    if (map_file_.empty()) {
      RCLCPP_FATAL(get_logger(), "Required parameter 'map_file' is empty.");
      throw std::runtime_error("map_file parameter not set");
    }

    std::vector<Point2D> map_points;
    try {
      map_points = loadMapFromCsv(map_file_);
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "Failed to load map: %s", e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu map points from '%s'",
                map_points.size(), map_file_.c_str());

    relocalizer_ = std::make_unique<GeometricRelocalizer>(map_points);
    matcher_ = std::make_unique<ScanToMapICP>(std::move(map_points), cfg);

    // map->odom offset starts at identity: until the first successful
    // match, corrected pose == raw odom pose.
    map_to_odom_offset_ = Pose2D{};

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanToMapICPNode::odomCallback, this,
                  std::placeholders::_1),
        sub_options);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanToMapICPNode::scanCallback, this,
                  std::placeholders::_1),
        sub_options);

    corrected_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        corrected_odom_topic_, rclcpp::QoS(10));

    confidence_pub_ = create_publisher<std_msgs::msg::Float64>(
        "~/confidence", rclcpp::QoS(10));

    relocalize_srv_ = create_service<std_srvs::srv::Trigger>(
        "relocalize",
        std::bind(&ScanToMapICPNode::relocalizeCallback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    RCLCPP_INFO(get_logger(),
                "scan_to_map_icp ready: scan_topic='%s' odom_topic='%s' -> "
                "'%s', map/odom/base frames = '%s'/'%s'/'%s'",
                scan_topic_.c_str(), odom_topic_.c_str(),
                corrected_odom_topic_.c_str(), map_frame_.c_str(),
                odom_frame_.c_str(), base_frame_.c_str());
  }

private:
private:
  void relocalizeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    RCLCPP_INFO(get_logger(), "Global relocalization service requested. Waiting for scan to compute pose...");
    
    std::unique_lock<std::mutex> lock(reloc_mutex_);
    reloc_done_ = false;
    trigger_global_relocalize_ = true;
    
    // Wait for scanCallback to solve the pose (timeout 15.0 seconds)
    if (reloc_cv_.wait_for(lock, std::chrono::seconds(15), [this]() { return reloc_done_; })) {
      response->success = true;
      // Format response message: x:<x>;y:<y>;theta:<theta>;confidence:<confidence>
      std::string msg = "x:" + std::to_string(reloc_result_pose_.x) +
                        ";y:" + std::to_string(reloc_result_pose_.y) +
                        ";theta:" + std::to_string(reloc_result_pose_.theta) +
                        ";confidence:" + std::to_string(reloc_result_confidence_);
      response->message = msg;
    } else {
      response->success = false;
      response->message = "Timeout waiting for laser scan or relocalizer to solve.";
      RCLCPP_ERROR(get_logger(), "Global relocalization service timeout.");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      current_odom_pose_ = poseFromOdomMsg(*msg);
      current_odom_twist_ = msg->twist;
    }
    have_odom_ = true;
  }

  // Looks up (and caches) the static extrinsic from the scan's frame to
  // base_frame_. LakiBeam mounts are rigid, so one lookup is enough; if the
  // transform isn't available yet (TF not warmed up), returns false and the
  // scan is skipped for that callback only.
  bool ensureLaserToBaseTransform(const std::string &scan_frame) {
    if (scan_frame == base_frame_) {
      laser_to_base_ = Pose2D{0.0, 0.0, 0.0};
      cached_scan_frame_ = scan_frame;
      return true;
    }

    if (laser_to_base_.has_value() && cached_scan_frame_ == scan_frame)
      return true;

    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
          base_frame_, scan_frame, tf2::TimePointZero,
          std::chrono::milliseconds(200));
      Pose2D p;
      p.x = t.transform.translation.x;
      p.y = t.transform.translation.y;
      p.theta = tf2::getYaw(t.transform.rotation);
      laser_to_base_ = p;
      cached_scan_frame_ = scan_frame;
      RCLCPP_INFO(get_logger(),
                  "Cached static transform %s -> %s (x=%.3f y=%.3f th=%.3f)",
                  scan_frame.c_str(), base_frame_.c_str(), p.x, p.y, p.theta);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Could not look up %s -> %s yet: %s",
                           scan_frame.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!have_odom_)
      return; // need at least one odom sample first

    if (!active_ && !trigger_global_relocalize_)
      return; // Sleep wait state: ignore incoming scans until active

    if (!ensureLaserToBaseTransform(msg->header.frame_id))
      return;
    const Pose2D &laser_to_base = *laser_to_base_;

    // --- look up time-synchronized odometry pose at scan timestamp ---
    Pose2D sync_odom_pose;
    try {
      geometry_msgs::msg::TransformStamped odom_to_base_tf = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, msg->header.stamp,
          std::chrono::milliseconds(50));
      sync_odom_pose.x = odom_to_base_tf.transform.translation.x;
      sync_odom_pose.y = odom_to_base_tf.transform.translation.y;
      sync_odom_pose.theta = tf2::getYaw(odom_to_base_tf.transform.rotation);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Failed to look up time-synchronized odom pose at scan timestamp: %s. Falling back to latest.",
                           ex.what());
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        sync_odom_pose = current_odom_pose_;
      }
    }

    // --- convert scan to Point2D in base_frame_ ---
    std::vector<Point2D> scan_points;
    scan_points.reserve(msg->ranges.size());
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
        continue;
      const double angle =
          msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const Point2D p_laser{r * std::cos(angle), r * std::sin(angle)};
      scan_points.push_back(laser_to_base.apply(p_laser));
    }
    if (scan_points.size() < 10) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Too few valid scan points (%zu), skipping this scan",
          scan_points.size());
      return;
    }

    // --- predict, then correct ---
    if (trigger_global_relocalize_) {
      RCLCPP_INFO(get_logger(), "Running global geometric relocalization...");
      auto start_reloc = std::chrono::high_resolution_clock::now();
      // Pass the current map-frame pose as the initial guess to help guide the search
      auto reloc_result = relocalizer_->relocalize(scan_points, *matcher_, map_to_odom_offset_ * sync_odom_pose);
      auto end_reloc = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> elapsed_reloc = end_reloc - start_reloc;

      if (!reloc_result.has_value()) {
        RCLCPP_ERROR(get_logger(), "Global relocalization found no geometric match at all — keeping current offset.");
        {
          std::lock_guard<std::mutex> lock(reloc_mutex_);
          reloc_result_pose_ = Pose2D{0.0, 0.0, 0.0};
          reloc_result_confidence_ = 0.0;
          reloc_done_ = true;
          trigger_global_relocalize_ = false;
        }
        reloc_cv_.notify_all();
        return; // don't process this scan — no valid pose
      }
      Pose2D initial_pose = *reloc_result;
      
      map_to_odom_offset_ = initial_pose * sync_odom_pose.inverse();

      // Transform scan points to map frame to compute coverage
      std::vector<Point2D> transformed_scan;
      transformed_scan.reserve(scan_points.size());
      for (const auto &sp : scan_points) {
        transformed_scan.push_back(initial_pose.apply(sp));
      }
      int covered_count = 0;
      const auto &map_pts = matcher_->mapPoints();
      for (const auto &mp : map_pts) {
        double min_dist_sq = std::numeric_limits<double>::max();
        for (const auto &tp : transformed_scan) {
          double dx = mp.x - tp.x;
          double dy = mp.y - tp.y;
          double dist_sq = dx*dx + dy*dy;
          if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
          }
        }
        if (min_dist_sq < 0.0225) { // 0.15m
          covered_count++;
        }
      }
      double map_coverage = map_pts.empty() ? 0.0 : (double)covered_count / map_pts.size();

      {
        std::lock_guard<std::mutex> lock(reloc_mutex_);
        reloc_result_pose_ = initial_pose;
        reloc_result_confidence_ = map_coverage;
        reloc_done_ = true;
        trigger_global_relocalize_ = false;
        active_ = true;
      }
      reloc_cv_.notify_all();

      RCLCPP_INFO(get_logger(), "Global relocalization complete in %.3f ms. Initial pose guess set to: [%.3f, %.3f, %.3f], confidence: %.1f%%",
                  elapsed_reloc.count(), initial_pose.x, initial_pose.y, initial_pose.theta, map_coverage * 100.0);
    }

    const Pose2D initial_guess = map_to_odom_offset_ * sync_odom_pose;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    const ICPResult result = matcher_->align(scan_points, initial_guess);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

    // Transform scan points to map frame once
    std::vector<Point2D> transformed_scan;
    transformed_scan.reserve(scan_points.size());
    for (const auto &sp : scan_points) {
      transformed_scan.push_back(result.corrected_pose.apply(sp));
    }

    // Calculate map coverage (within 15 cm of any scan point)
    int covered_count = 0;
    const auto &map_pts = matcher_->mapPoints();
    for (const auto &mp : map_pts) {
      double min_dist_sq = std::numeric_limits<double>::max();
      for (const auto &tp : transformed_scan) {
        double dx = mp.x - tp.x;
        double dy = mp.y - tp.y;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
        }
      }
      if (min_dist_sq < 0.0225) { // 0.15m threshold -> 0.15^2 = 0.0225
        covered_count++;
      }
    }
    double map_coverage = map_pts.empty() ? 0.0 : (double)covered_count / map_pts.size();

    bool offset_updated = false;

    const bool is_good_match = (result.inlier_rms < max_accepted_rms_ && map_coverage >= 0.70 && result.inlier_count >= 5);

    if (is_good_match) {
      offset_updated = true;
      consecutive_failures_ = 0;
      map_to_odom_offset_ =
          result.corrected_pose * sync_odom_pose.inverse();
      double inlier_ratio = (double)result.inlier_count / scan_points.size();

      RCLCPP_DEBUG(get_logger(), "ICP alignment took %.3f ms (iters=%d) | Map Coverage: %.1f%% | Scan Inliers: %.1f%% | RMS: %.2f cm",
                   elapsed.count(), result.iterations, map_coverage * 100.0, inlier_ratio * 100.0, result.inlier_rms * 100.0);
    } else {
      offset_updated = false;
      consecutive_failures_++;
      RCLCPP_WARN(get_logger(), "ICP failed (fail #%d/%d): converged=%d, coverage=%.1f%%, RMS=%.4f. %s",
                  consecutive_failures_, fallback_hysteresis_,
                  result.converged, map_coverage * 100.0, result.inlier_rms,
                  consecutive_failures_ >= fallback_hysteresis_
                      ? "Triggering fallback relocalization..."
                      : "Skipping fallback — waiting for more consecutive failures.");

      // offset_updated declared above; will be set to true by whichever
      // fallback tier succeeds

      // --- Tier 1 fallback: scan-to-scan ICP against the frozen reference ---
      // Uses the persistent s2s_matcher_ whose KD-tree is rebuilt only on success
      // frames.  On consecutive failures the reference stays frozen so the tree
      // costs nothing to reuse.
      if (s2s_matcher_ != nullptr &&
          consecutive_failures_ >= scan_to_scan_hysteresis_) {

        auto s2s_start = std::chrono::high_resolution_clock::now();

        try {
          Pose2D s2s_initial_guess = s2s_reference_odom_.inverse() * sync_odom_pose;
          ICPResult s2s_result = s2s_matcher_->align(scan_points, s2s_initial_guess);

          auto s2s_end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double, std::milli> s2s_elapsed = s2s_end - s2s_start;

          if (s2s_result.inlier_rms < max_accepted_rms_ && s2s_result.inlier_count >= 10) {
            Pose2D rel_delta = s2s_result.corrected_pose;
            Pose2D ref_map_pose = map_to_odom_offset_ * s2s_reference_odom_;
            Pose2D current_map_pose = ref_map_pose * rel_delta;

            map_to_odom_offset_ = current_map_pose * sync_odom_pose.inverse();
            consecutive_failures_ = 0;
            offset_updated = true;

            RCLCPP_INFO(get_logger(),
                        "Scan-to-scan ICP fallback succeeded in %.2f ms: "
                        "rel=[%.3f, %.3f, %.3f], rms=%.4f, inliers=%d",
                        s2s_elapsed.count(),
                        rel_delta.x, rel_delta.y, rel_delta.theta,
                        s2s_result.inlier_rms, s2s_result.inlier_count);

            int cov_count = 0;
            for (const auto &mp : map_pts) {
              double min_dist_sq = std::numeric_limits<double>::max();
              for (const auto &sp : scan_points) {
                Point2D tp = current_map_pose.apply(sp);
                double dx = mp.x - tp.x;
                double dy = mp.y - tp.y;
                double dist_sq = dx*dx + dy*dy;
                if (dist_sq < min_dist_sq) min_dist_sq = dist_sq;
              }
              if (min_dist_sq < 0.0225) cov_count++;
            }
            map_coverage = map_pts.empty() ? 0.0 : (double)cov_count / map_pts.size();
          } else {
            RCLCPP_DEBUG(get_logger(), "Scan-to-scan ICP fell through: rms=%.4f, inliers=%d (%.1f ms)",
                         s2s_result.inlier_rms, s2s_result.inlier_count, s2s_elapsed.count());
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Scan-to-scan ICP exception: %s", e.what());
        }
      }

      // --- Tier 2 fallback: geometric relocalization against the static map ---
      if (!offset_updated && consecutive_failures_ >= fallback_hysteresis_) {
        const Pose2D map_frame_initial_guess = map_to_odom_offset_ * sync_odom_pose;
        auto reloc_result = relocalizer_->relocalize(scan_points, *matcher_, map_frame_initial_guess);

        if (!reloc_result.has_value()) {
          RCLCPP_ERROR(get_logger(), "Fallback relocalization found no match at all - keeping previous map->odom offset.");
        } else {
          Pose2D reloc_pose = *reloc_result;
          // Calculate map coverage of the relocalized pose
          std::vector<Point2D> transformed_scan;
          transformed_scan.reserve(scan_points.size());
          for (const auto &sp : scan_points) {
            transformed_scan.push_back(reloc_pose.apply(sp));
          }
          int covered_count = 0;
          const auto &map_pts = matcher_->mapPoints();
          for (const auto &mp : map_pts) {
            double min_dist_sq = std::numeric_limits<double>::max();
            for (const auto &tp : transformed_scan) {
              double dx = mp.x - tp.x;
              double dy = mp.y - tp.y;
              double dist_sq = dx*dx + dy*dy;
              if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
              }
            }
            if (min_dist_sq < 0.0225) {
              covered_count++;
            }
          }
          double reloc_coverage = map_pts.empty() ? 0.0 : (double)covered_count / map_pts.size();

          if (reloc_coverage > map_coverage && reloc_coverage >= 0.70) {
            RCLCPP_INFO(get_logger(), "Fallback relocalization succeeded. Recovered pose: [%.3f, %.3f, %.3f], coverage: %.1f%%",
                        reloc_pose.x, reloc_pose.y, reloc_pose.theta, reloc_coverage * 100.0);
            map_to_odom_offset_ = reloc_pose * sync_odom_pose.inverse();
            map_coverage = reloc_coverage;
            consecutive_failures_ = 0;
            offset_updated = true;
          } else {
            RCLCPP_ERROR(get_logger(), "Fallback relocalization did not find a better fit (reloc coverage=%.1f%%, original map coverage=%.1f%%). Keeping previous map->odom offset.",
                         reloc_coverage * 100.0, map_coverage * 100.0);
          }
        }
      }

    }

    const Pose2D corrected_pose = map_to_odom_offset_ * sync_odom_pose;
    publishCorrectedOdom(msg->header.stamp, corrected_pose, map_coverage);
    broadcastMapToOdom(msg->header.stamp);

    // Rebuild persistent scan-to-scan matcher only on success frames.
    // Freezing the reference here means the KD-tree + normals survive
    // across consecutive failures without re-computation.
    if (offset_updated && scan_points.size() >= 10) {
      ICPConfig s2s_cfg;
      s2s_cfg.max_correspondence_dist = 0.5;
      s2s_cfg.max_iterations = 20;
      s2s_cfg.min_inlier_ratio = 0.15;
      s2s_cfg.huber_delta = 0.10;
      s2s_cfg.normal_k_neighbors = 5;
      s2s_cfg.translation_eps = 1e-4;
      s2s_cfg.rotation_eps = 1e-5;
      s2s_cfg.verbose = false;
      s2s_matcher_ = std::make_unique<ScanToMapICP>(scan_points, s2s_cfg);
      s2s_reference_odom_ = sync_odom_pose;
    }
  }

  void publishCorrectedOdom(const rclcpp::Time &stamp, const Pose2D &pose, double map_coverage) {
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = map_frame_;
    out.child_frame_id = base_frame_;
    out.pose.pose.position.x = pose.x;
    out.pose.pose.position.y = pose.y;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = yawToQuaternion(pose.theta);
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      out.twist = current_odom_twist_; // pass wheel-odometry-derived velocity
                                       // through unchanged
    }

    // Scale covariance based on map coverage (accuracy representation)
    double var = 0.001 / (map_coverage * map_coverage + 1e-4);
    out.pose.covariance[0] = var;   // x variance
    out.pose.covariance[7] = var;   // y variance
    out.pose.covariance[35] = var;  // yaw variance

    // Store the raw map_coverage (confidence) in the unused z covariance element
    out.pose.covariance[14] = map_coverage;

    corrected_odom_pub_->publish(out);

    std_msgs::msg::Float64 conf_msg;
    conf_msg.data = map_coverage;
    confidence_pub_->publish(conf_msg);
  }

  void broadcastMapToOdom(const rclcpp::Time &stamp) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;
    t.transform.translation.x = map_to_odom_offset_.x;
    t.transform.translation.y = map_to_odom_offset_.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = yawToQuaternion(map_to_odom_offset_.theta);
    tf_broadcaster_->sendTransform(t);
  }

  // --- parameters ---
  std::string map_file_, scan_topic_, odom_topic_, corrected_odom_topic_;
  std::string map_frame_, odom_frame_, base_frame_;
  double max_accepted_rms_;
  int fallback_hysteresis_;
  int scan_to_scan_hysteresis_;

  // --- state ---
  std::unique_ptr<GeometricRelocalizer> relocalizer_;
  std::unique_ptr<ScanToMapICP> matcher_;
  Pose2D map_to_odom_offset_;
  Pose2D current_odom_pose_;
  geometry_msgs::msg::TwistWithCovariance current_odom_twist_;
  bool have_odom_ = false;
  int consecutive_failures_ = 0;

  // --- Persistent scan-to-scan ICP fallback ---
  // The KD-tree + normals inside s2s_matcher_ are rebuilt only when a
  // success frame provides a new reference scan, NOT on every fallback attempt.
  std::unique_ptr<ScanToMapICP> s2s_matcher_;
  Pose2D s2s_reference_odom_;

  std::optional<Pose2D> laser_to_base_;
  std::string cached_scan_frame_;

  // --- ROS interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr confidence_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::atomic<bool> trigger_global_relocalize_{false};
  std::atomic<bool> active_{false};
  std::mutex reloc_mutex_;
  std::mutex odom_mutex_;
  std::condition_variable reloc_cv_;
  bool reloc_done_ = false;
  Pose2D reloc_result_pose_;
  double reloc_result_confidence_ = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanToMapICPNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}