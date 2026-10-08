// board_pose_detector.hpp
//
// AprilTag 25h9 marker-board pose tap for a head camera stream, on its own
// thread so detection never stalls the encode loop that feeds it.
//
// Same detector, board layout and PnP gating as ffw_depthai's
// depthai_720p_raw_streamer.cpp (the OAK-D tap): ~5 Hz wall-clock gate, one
// combined-corner solvePnP seeded from the last accepted pose, sanity + jump
// rejection with forced reseed after 3 consecutive rejects, publish only when
// >= 3 board tags are matched. Publishes, under `ns`:
//   <ns>/marker_board_pose_camera_frame  T_camera_board (frame `camera_frame`)
//   <ns>/marker_board_pose               T_base_board via /head_camera_tf
//                                        (dropped if that tf is > 0.5 s old)
//   <ns>/apriltag_telemetry              "<cam>_fps=.. apriltag_fps=.. avg_margin=..
//                                         num_tags=.. reproj_px=.."
//   <ns>/apriltag_detections             one JSON per detection pass (calibration
//                                         recorders): stamp, image size, every tag's
//                                         id/margin/4 raw pixel corners, used board
//                                         tags, reprojection, and this pass's accepted
//                                         T_camera_board (null if none published)
//
// /head_camera_tf is the resolved head_camera_frame -> base_link transform;
// its mount offset was calibrated for the OAK-D, so the base_link pose is only
// as right as that calibration is for the camera actually mounted.

#pragma once

#include <apriltag.h>
#include <tag25h9.h>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace ffw_stream {

// ── AprilTag board layout (25h9) -- copy of depthai_720p_raw_streamer.cpp ──
// Calibrated by ~/utilities_ws/src/apriltag_25h9_cpp's bundle_adjustment
// (board_layout_optimized.yaml + tags_25h9.yaml). Board frame = tag 1's pose;
// (x,y,z) + axis-angle (rx,ry,rz) relative to it; size = edge length [m].
struct BoardTag {
  int id;
  double size;
  double x, y, z;
  double rx, ry, rz;
};

inline const BoardTag kBoardLayout[] = {
  {1,  0.057,  0.0,                    0.0,                    0.0,
       0.0,                    0.0,                    0.0},
  {2,  0.057,  0.18355213452308197,   -0.0005495590273141514,  0.0,
       0.003392793153333883,  -0.001652768831466606,   0.005596125526348661},
  {3,  0.057, -0.002604290279393214,   0.18262499145172836,    0.0,
       0.0022404711209055793, -0.0017038086815047686,  0.01313233381601511},
  {4,  0.057, -0.1821974196064706,    -0.0016526647265309707,  0.0,
      -0.0005940458783676034,  0.003384338795834879,   0.0077421718916435315},
  {5,  0.041,  0.05777201673670586,   -0.19155240115435038,    0.0,
       0.0024879033811299974, -0.0017303780467724601,  0.020033850859068666},
  {7,  0.041, -0.0903143682118519,    -0.09851627871582525,    0.0,
      -0.003121444688450445,   0.002343849457630141,  -0.009063486113326337},
  {8,  0.041,  0.08846322550535259,   -0.09928499260033768,    0.0,
      -0.0041147398871153335,  0.0019800553991998655,  0.010351065748505018},
  {9,  0.041, -0.058080502405420564,  -0.19258298833310175,    0.0,
      -0.0011850492009372472,  0.0012848023973317715,  0.000540310454424208},
  {20, 0.015, -0.033553076106144185,  -0.2921062756094909,     0.025,
      -0.0010910218728025943,  0.0030283245402257302,  0.00023589369988523032},
  {25, 0.105, -0.17414292814679452,    0.16938160622453025,    0.0,
      -0.002828623660624173,   0.001109427942824287,   0.01576257580464725},
  {30, 0.015,  0.04296011055731651,   -0.29273904961469743,    0.025,
      -0.008045401913811596,   0.01695414816661725,   -0.01750620945015386},
  {31, 0.015, -0.0852440748546845,    -0.28330018814973335,    0.01,
      -0.0344126233894834,    -0.015019776036531377,  -0.005325592433057143},
  {32, 0.015,  0.09416774058800953,   -0.2862785785077211,     0.01,
      -0.022274917428141635,  -0.013421948142595262,  -0.017686629412183158},
};

// apriltag_detection_t::p[0..3] corner order/winding.
inline std::array<cv::Point3f, 4> tagLocalCorners(double size) {
  float h = static_cast<float>(size / 2.0);
  return {cv::Point3f(-h, -h, 0.0f), cv::Point3f(h, -h, 0.0f),
          cv::Point3f(h, h, 0.0f), cv::Point3f(-h, h, 0.0f)};
}

inline std::map<int, std::array<cv::Point3f, 4>> buildBoardCorners() {
  std::map<int, std::array<cv::Point3f, 4>> out;
  for (const auto &t : kBoardLayout) {
    Eigen::Vector3d aa(t.rx, t.ry, t.rz);
    double angle = aa.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (angle > 1e-12) R = Eigen::AngleAxisd(angle, aa / angle).toRotationMatrix();
    Eigen::Vector3d trans(t.x, t.y, t.z);
    auto local = tagLocalCorners(t.size);
    std::array<cv::Point3f, 4> pts;
    for (int k = 0; k < 4; ++k) {
      Eigen::Vector3d pb = R * Eigen::Vector3d(local[k].x, local[k].y, local[k].z) + trans;
      pts[k] = cv::Point3f(static_cast<float>(pb.x()), static_cast<float>(pb.y()),
                           static_cast<float>(pb.z()));
    }
    out[t.id] = pts;
  }
  return out;
}

class BoardPoseDetector {
 public:
  // ns e.g. "/d435"; cam_label prefixes the telemetry fps field ("d435").
  BoardPoseDetector(rclcpp::Node::SharedPtr node, const std::string &ns,
                    const std::string &camera_frame, const std::string &cam_label,
                    double period_s = 0.2)
      : node_(std::move(node)), camera_frame_(camera_frame), cam_label_(cam_label),
        period_s_(period_s), board_corners_(buildBoardCorners()) {
    family_ = tag25h9_create();
    detector_ = apriltag_detector_create();
    apriltag_detector_add_family(detector_, family_);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        ns + "/marker_board_pose", rclcpp::QoS(1).best_effort());
    pose_cam_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        ns + "/marker_board_pose_camera_frame", rclcpp::QoS(1).best_effort());
    telemetry_pub_ = node_->create_publisher<std_msgs::msg::String>(
        ns + "/apriltag_telemetry", rclcpp::QoS(1).best_effort());
    detections_pub_ = node_->create_publisher<std_msgs::msg::String>(
        ns + "/apriltag_detections", rclcpp::QoS(5).best_effort());
    cam_tf_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/head_camera_tf", rclcpp::QoS(1).transient_local().reliable(),
        [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(tf_mtx_);
          cam_tf_ = *msg;
          have_cam_tf_ = true;
        });
    last_submit_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    last_fps_time_ = std::chrono::steady_clock::now();
    worker_ = std::thread(&BoardPoseDetector::run, this);
  }

  ~BoardPoseDetector() {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    apriltag_detector_destroy(detector_);
    tag25h9_destroy(family_);
  }

  BoardPoseDetector(const BoardPoseDetector &) = delete;
  BoardPoseDetector &operator=(const BoardPoseDetector &) = delete;

  void set_intrinsics(const std::array<double, 9> &K, const std::vector<double> &D) {
    std::lock_guard<std::mutex> lk(mtx_);
    K_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) K_.at<double>(i / 3, i % 3) = K[i];
    D_ = cv::Mat(1, static_cast<int>(D.size()), CV_64F);
    for (size_t i = 0; i < D.size(); ++i) D_.at<double>(0, static_cast<int>(i)) = D[i];
  }

  // Encoder-side frame rate, reported in telemetry as "<cam_label>_fps".
  void set_camera_fps(double fps) { cam_fps_.store(fps); }

  // Called from the capture loop for every frame. Cheap unless a detection is
  // due (~period_s) and the worker is idle: then the RGB8 frame is reduced to
  // luma into the worker's buffer and the worker is woken. Never blocks on the
  // detector -- a busy worker just means this frame is skipped.
  // cap_stamp: the frame's capture time in seconds on the ROS system clock
  // (RealSense global/system timestamp), carried into apriltag_detections as
  // "cap_stamp" so recorders can sample joint states at the exposure, not at
  // detection end.
  void submit_rgb(const uint8_t *rgb, int w, int h, int stride_bytes, double cap_stamp = 0.0) {
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_submit_).count() < period_s_) return;
    std::unique_lock<std::mutex> lk(mtx_, std::try_to_lock);
    if (!lk.owns_lock() || pending_ || K_.empty()) return;
    last_submit_ = now;
    gray_.resize(static_cast<size_t>(w) * h);
    for (int y = 0; y < h; ++y) {
      const uint8_t *src = rgb + static_cast<size_t>(y) * stride_bytes;
      uint8_t *dst = gray_.data() + static_cast<size_t>(y) * w;
      for (int x = 0; x < w; ++x) {
        // ITU-R BT.601 luma, integer: (77 R + 150 G + 29 B) >> 8
        dst[x] = static_cast<uint8_t>((77 * src[3 * x] + 150 * src[3 * x + 1] +
                                       29 * src[3 * x + 2]) >> 8);
      }
    }
    gw_ = w;
    gcap_ = cap_stamp;
    gh_ = h;
    pending_ = true;
    lk.unlock();
    cv_.notify_one();
  }

 private:
  void run() {
    std::vector<uint8_t> gray;
    int w = 0, h = 0;
    double cap = 0.0;
    cv::Mat K, D;
    while (true) {
      {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this] { return stop_ || pending_; });
        if (stop_) return;
        gray.swap(gray_);
        w = gw_;
        cap = gcap_;
        h = gh_;
        K = K_.clone();
        D = D_.clone();
      }
      detect_once(gray, w, h, K, D, cap);
      {
        std::lock_guard<std::mutex> lk(mtx_);
        pending_ = false;
      }
    }
  }

  void detect_once(const std::vector<uint8_t> &gray, int w, int h, const cv::Mat &K,
                   const cv::Mat &D, double cap_stamp) {
    image_u8_t *im = image_u8_create(w, h);  // aligned stride for apriltag's SIMD
    for (int row = 0; row < h; ++row) {
      std::memcpy(im->buf + row * im->stride, gray.data() + static_cast<size_t>(row) * w, w);
    }
    zarray_t *detections = apriltag_detector_detect(detector_, im);
    int n = zarray_size(detections);
    double margin_sum = 0.0;
    std::ostringstream tags_json;
    tags_json << std::setprecision(6);
    std::vector<cv::Point3f> obj_pts;
    std::vector<cv::Point2f> img_pts;
    for (int i = 0; i < n; ++i) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      margin_sum += det->decision_margin;
      tags_json << (i ? "," : "") << "[" << det->id << "," << det->decision_margin;
      for (int k = 0; k < 4; ++k) tags_json << "," << det->p[k][0] << "," << det->p[k][1];
      tags_json << "]";
      auto it = board_corners_.find(det->id);
      if (it != board_corners_.end()) {
        for (int k = 0; k < 4; ++k) {
          obj_pts.push_back(it->second[k]);
          img_pts.emplace_back(static_cast<float>(det->p[k][0]),
                               static_cast<float>(det->p[k][1]));
        }
      }
    }
    double avg_margin = (n > 0) ? margin_sum / n : 0.0;
    size_t used_tags = obj_pts.size() / 4;
    apriltag_detections_destroy(detections);
    image_u8_destroy(im);

    published_ = false;
    if (obj_pts.size() >= 4) solve_and_publish(obj_pts, img_pts, K, D, used_tags);

    {
      std_msgs::msg::String js;
      std::ostringstream o;
      o << std::fixed << std::setprecision(6) << "{\"stamp\":" << node_->now().seconds()
        << ",\"cap_stamp\":" << cap_stamp << std::defaultfloat << std::setprecision(9)
        << ",\"w\":" << w
        << ",\"h\":" << h << ",\"num_tags\":" << n << ",\"used_tags\":" << used_tags
        << ",\"reproj_px\":" << last_reproj_px_ << ",\"pose\":";
      if (published_) {
        o << "{\"t\":[" << pub_t_.x() << "," << pub_t_.y() << "," << pub_t_.z() << "],\"q\":["
          << pub_q_.w() << "," << pub_q_.x() << "," << pub_q_.y() << "," << pub_q_.z() << "]}";
      } else {
        o << "null";
      }
      o << ",\"tags\":[" << tags_json.str() << "]}";  // [id, margin, u0,v0, .., u3,v3]
      js.data = o.str();
      detections_pub_->publish(js);
    }

    auto now = std::chrono::steady_clock::now();
    pass_count_++;
    double el = std::chrono::duration<double>(now - last_fps_time_).count();
    if (el >= 1.0) {
      apriltag_fps_ = pass_count_ / el;
      pass_count_ = 0;
      last_fps_time_ = now;
    }
    std_msgs::msg::String tel;
    std::ostringstream ss;
    ss << cam_label_ << "_fps=" << std::fixed << std::setprecision(1) << cam_fps_.load()
       << " apriltag_fps=" << std::setprecision(1) << apriltag_fps_
       << " avg_margin=" << std::setprecision(1) << avg_margin << " num_tags=" << n
       << " reproj_px=" << std::setprecision(2) << last_reproj_px_;
    tel.data = ss.str();
    telemetry_pub_->publish(tel);
  }

  static Eigen::Matrix3d rvecToR(const cv::Mat &rv) {
    Eigen::Vector3d v(rv.at<double>(0), rv.at<double>(1), rv.at<double>(2));
    double a = v.norm();
    return (a > 1e-12) ? Eigen::AngleAxisd(a, v / a).toRotationMatrix()
                       : Eigen::Matrix3d::Identity();
  }

  void solve_and_publish(const std::vector<cv::Point3f> &obj_pts,
                         const std::vector<cv::Point2f> &img_pts, const cv::Mat &K,
                         const cv::Mat &D, size_t used_tags) {
    // Solve into trial vars: a near-planar board has two plausible poses and a
    // seeded ITERATIVE solve can flip -- check before accepting as the seed.
    // Unseeded: SQPNP (global, picks the in-front solution). Seeded: cheap
    // ITERATIVE from the last accepted pose; if that lands behind the camera,
    // retry SQPNP unseeded. A pinhole cannot tell a board in front from its
    // mirror behind (same projections, same reprojection error), so the
    // t.z > 0 chirality check is part of "sane" -- an unseeded ITERATIVE solve
    // on a synthetic fronto-parallel board converged to t.z = -0.9 m.
    constexpr double kMaxPlausibleDistM = 5.0;
    auto sane = [&](const cv::Mat &r, const cv::Mat &t) {
      for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(t.at<double>(i)) || !std::isfinite(r.at<double>(i))) return false;
      }
      return t.at<double>(2) > 0.0 && cv::norm(t) < kMaxPlausibleDistM;
    };
    cv::Mat trial_r = pnp_rvec_.clone(), trial_t = pnp_tvec_.clone();
    bool ok = cv::solvePnP(obj_pts, img_pts, K, D, trial_r, trial_t, pnp_seeded_,
                           pnp_seeded_ ? cv::SOLVEPNP_ITERATIVE : cv::SOLVEPNP_SQPNP);
    ok = ok && sane(trial_r, trial_t);
    if (!ok && pnp_seeded_) {
      trial_r.release();
      trial_t.release();
      ok = cv::solvePnP(obj_pts, img_pts, K, D, trial_r, trial_t, false, cv::SOLVEPNP_SQPNP) &&
           sane(trial_r, trial_t);
    }
    if (!ok) return;

    constexpr double kMaxJumpDistM = 0.05;     // per ~200 ms pass
    constexpr double kMaxJumpAngleRad = 0.26;  // ~15 deg
    constexpr int kMaxConsecutiveRejects = 3;  // sustained = real motion -> reseed
    bool accept = true;
    if (pnp_seeded_) {
      double dt = cv::norm(trial_t - pnp_tvec_);
      double da = Eigen::AngleAxisd(rvecToR(trial_r) * rvecToR(pnp_rvec_).transpose()).angle();
      if (dt > kMaxJumpDistM || std::abs(da) > kMaxJumpAngleRad) {
        accept = (++consecutive_rejects_ >= kMaxConsecutiveRejects);
      }
    }
    if (accept) {
      pnp_rvec_ = trial_r;
      pnp_tvec_ = trial_t;
      pnp_seeded_ = true;
      consecutive_rejects_ = 0;
    }
    if (!pnp_seeded_) return;

    std::vector<cv::Point2f> reproj;
    cv::projectPoints(obj_pts, pnp_rvec_, pnp_tvec_, K, D, reproj);
    double err = 0.0;
    for (size_t i = 0; i < reproj.size(); ++i) err += cv::norm(reproj[i] - img_pts[i]);
    last_reproj_px_ = err / reproj.size();

    constexpr size_t kMinTagsToPublish = 3;  // 1-2 tags = flip-prone regime
    if (used_tags < kMinTagsToPublish) return;

    Eigen::Matrix3d R_cb = rvecToR(pnp_rvec_);
    Eigen::Vector3d t_cb(pnp_tvec_.at<double>(0), pnp_tvec_.at<double>(1),
                         pnp_tvec_.at<double>(2));
    auto stamp = node_->now();
    pose_cam_pub_->publish(make_pose(stamp, camera_frame_, R_cb, t_cb));
    published_ = true;
    pub_t_ = t_cb;
    pub_q_ = Eigen::Quaterniond(R_cb);

    geometry_msgs::msg::TransformStamped tf;
    {
      std::lock_guard<std::mutex> lk(tf_mtx_);
      if (!have_cam_tf_) return;
      tf = cam_tf_;
    }
    constexpr double kCamTfMaxAgeS = 0.5;
    if ((stamp - rclcpp::Time(tf.header.stamp)).seconds() > kCamTfMaxAgeS) return;
    const auto &ct = tf.transform;
    Eigen::Matrix3d R_bc = Eigen::Quaterniond(ct.rotation.w, ct.rotation.x, ct.rotation.y,
                                              ct.rotation.z).toRotationMatrix();
    Eigen::Vector3d t_bc(ct.translation.x, ct.translation.y, ct.translation.z);
    pose_pub_->publish(make_pose(stamp, "base_link", R_bc * R_cb, R_bc * t_cb + t_bc));
  }

  static geometry_msgs::msg::PoseStamped make_pose(const rclcpp::Time &stamp,
                                                   const std::string &frame,
                                                   const Eigen::Matrix3d &R,
                                                   const Eigen::Vector3d &t) {
    geometry_msgs::msg::PoseStamped m;
    m.header.stamp = stamp;
    m.header.frame_id = frame;
    m.pose.position.x = t.x();
    m.pose.position.y = t.y();
    m.pose.position.z = t.z();
    Eigen::Quaterniond q(R);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    return m;
  }

  rclcpp::Node::SharedPtr node_;
  std::string camera_frame_, cam_label_;
  double period_s_;
  const std::map<int, std::array<cv::Point3f, 4>> board_corners_;
  apriltag_family_t *family_ = nullptr;
  apriltag_detector_t *detector_ = nullptr;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_, pose_cam_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_, detections_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cam_tf_sub_;
  std::mutex tf_mtx_;
  geometry_msgs::msg::TransformStamped cam_tf_;
  bool have_cam_tf_ = false;

  // capture -> worker handoff
  std::mutex mtx_;
  std::condition_variable cv_;
  bool pending_ = false, stop_ = false;
  std::vector<uint8_t> gray_;
  int gw_ = 0, gh_ = 0;
  double gcap_ = 0.0;
  cv::Mat K_, D_;
  std::chrono::steady_clock::time_point last_submit_;
  std::thread worker_;

  // worker-only state
  cv::Mat pnp_rvec_, pnp_tvec_;
  bool pnp_seeded_ = false;
  int consecutive_rejects_ = 0;
  double last_reproj_px_ = -1.0;
  bool published_ = false;          // this pass published a camera-frame pose
  Eigen::Vector3d pub_t_;
  Eigen::Quaterniond pub_q_;
  int pass_count_ = 0;
  double apriltag_fps_ = 0.0;
  std::chrono::steady_clock::time_point last_fps_time_;
  std::atomic<double> cam_fps_{0.0};
};

}  // namespace ffw_stream
