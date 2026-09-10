// depthai_720p_raw_streamer.cpp
//
// Clone of depthai_node.cpp (the 1080p HW-encoded OAK-D streamer, kept as fallback),
// with the on-device VideoEncoder removed: the OAK-D pushes RAW NV12 720p frames
// over USB and the HOST CPU encodes them with x264enc (GStreamer), at a higher
// bitrate (~20 Mbps) than the OAK-D HW encoder's 8 Mbps CBR ceiling.
//
// Pipeline mirrors ffw_stream/realsense_udp_streamer.cpp:create_gst_stream():
//   appsrc(NV12) -> videoconvert(I420) -> x264enc(veryfast, zerolatency, bitrate)
//   -> h264parse(config-interval=-1) -> rtph264pay(pt=96) -> udpsink
//
// Args (positional, after ROS args): <dest_ip> <video_port> <fps> <bitrate_kbps>
//   dest_ip      default 192.168.0.241
//   video_port   default 9110  (the 1080p fallback owns 9100)
//   fps          default 30
//   bitrate_kbps default 20000 (20 Mbps)
//
// Telemetry (FPS / worst-delay / host clock TW) goes to video_port+200, same
// format as the fallback so the unified receiver can show it.

#include "depthai/depthai.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <memory>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <cstdlib>
#include <algorithm>
#include <array>
#include <map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

// apriltag.h / tag25h9.h already self-guard with extern "C" internally.
#include <apriltag.h>
#include <tag25h9.h>

void sendUdpText(int sock, const struct sockaddr_in &addr, const std::string &msg) {
  if (sock >= 0) {
    sendto(sock, msg.c_str(), msg.length(), 0,
           reinterpret_cast<const struct sockaddr *>(&addr), sizeof(addr));
  }
}

void printSystemInformation(const dai::SystemInformation &info, int sock,
                            const struct sockaddr_in &addr) {
  const float m = 1024.0f * 1024.0f;
  const auto &t = info.chipTemperature;
  std::ostringstream ss;
  ss << "[OAK-720p] "
     << "CPU CSS: " << std::fixed << std::setprecision(1)
     << info.leonCssCpuUsage.average * 100.0f << "% | "
     << "MSS: " << info.leonMssCpuUsage.average * 100.0f << "% | "
     << "RAM: " << info.ddrMemoryUsage.used / m << "/"
     << info.ddrMemoryUsage.total / m << " MiB | "
     << "Temp: " << t.average << "C";
  sendUdpText(sock, addr, ss.str());
}

// ── AprilTag board layout (25h9, tags 1..32) ────────────────────────────────
// Calibrated by ~/utilities_ws/src/apriltag_25h9_cpp's bundle_adjustment
// tooling (config/board_layout_optimized.yaml + config/tags_25h9.yaml) --
// same physical board this node detects (confirmed: tag IDs match). Board
// frame origin/orientation = tag 1's pose (tag 1 has identity pose below);
// every other tag's (x,y,z) + axis-angle rotation (rx,ry,rz) is relative to
// that. Sizes are each tag's physical edge length in metres.
struct BoardTag {
  int id;
  double size;
  double x, y, z;
  double rx, ry, rz;
};

static const BoardTag kBoardLayout[] = {
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

// Standard AprilTag object-frame corner order/winding -- matches
// apriltag_detection_t::p[0..3] exactly (the same convention
// apriltag_pose.c's own estimate_tag_pose() uses internally), so pairing
// kBoardLayout-derived object points with det->p[] pixel points index-for-
// index is a correct correspondence for solvePnP.
static std::array<cv::Point3f, 4> tagLocalCorners(double size) {
  float h = static_cast<float>(size / 2.0);
  return {cv::Point3f(-h, -h, 0.0f), cv::Point3f(h, -h, 0.0f),
          cv::Point3f(h, h, 0.0f), cv::Point3f(-h, h, 0.0f)};
}

// Rotates+translates each tag's local corners into the shared board frame.
static std::map<int, std::array<cv::Point3f, 4>> buildBoardCorners() {
  std::map<int, std::array<cv::Point3f, 4>> out;
  for (const auto &t : kBoardLayout) {
    Eigen::Vector3d aa(t.rx, t.ry, t.rz);
    double angle = aa.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (angle > 1e-12) {
      R = Eigen::AngleAxisd(angle, aa / angle).toRotationMatrix();
    }
    Eigen::Vector3d trans(t.x, t.y, t.z);
    auto local = tagLocalCorners(t.size);
    std::array<cv::Point3f, 4> board_pts;
    for (int k = 0; k < 4; ++k) {
      Eigen::Vector3d p(local[k].x, local[k].y, local[k].z);
      Eigen::Vector3d pb = R * p + trans;
      board_pts[k] = cv::Point3f(static_cast<float>(pb.x()),
                                 static_cast<float>(pb.y()),
                                 static_cast<float>(pb.z()));
    }
    out[t.id] = board_pts;
  }
  return out;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("depthai_720p_raw_udp_node");

  // ── Positional args (fall back to defaults if not supplied) ─────────────
  const std::string dest_ip   = (argc > 1) ? argv[1] : "192.168.0.241";
  const int  video_port       = (argc > 2) ? std::atoi(argv[2]) : 9110;
  const int  fps              = (argc > 3) ? std::atoi(argv[3]) : 15;
  const int  bitrate_kbps     = (argc > 4) ? std::atoi(argv[4]) : 20000;
  const int  telemetry_port   = video_port + 200;

  // Streamed output size — the one place to change resolution. Feeds
  // requestOutput(), the GStreamer caps, the startup log, and the CameraInfo
  // intrinsics query (the actual first-frame size is the authoritative one).
  const uint32_t k_out_width  = 1280;
  const uint32_t k_out_height = 720;

  gst_init(nullptr, nullptr);

  std::shared_ptr<dai::Device> device;
  try {
    device = std::make_shared<dai::Device>();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect to device: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(node->get_logger(),
              "USB Speed: %d  |  codec: host-CPU x264enc (ultrafast/zerolatency) @ %d kbps | %dx%d@%d",
              static_cast<int>(device->getUsbSpeed()), bitrate_kbps, k_out_width, k_out_height, fps);

  dai::Pipeline pipeline(device);

  auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
  // Sharpness 0: aruco markers are maximum-contrast binary grids — sharpening's
  // overshoot/ringing fakes secondary edges around the corners and breaks
  // detectMarkers. Disable it; the grid already has all the edge contrast it needs.
  cam->initialControl.setSharpness(0);
  cam->initialControl.setLumaDenoise(1);
  cam->initialControl.setChromaDenoise(1);

  // RAW NV12 720p — no on-device VideoEncoder. The host encodes it.
  auto *videoOut = cam->requestOutput({k_out_width, k_out_height}, dai::ImgFrame::Type::NV12,
                                      dai::ImgResizeMode::CROP, static_cast<float>(fps));

  // Queue depth = 1, blocking = false → always drop oldest, never accumulate latency
  auto videoQueue = videoOut->createOutputQueue(1, false);

  auto sysLog = pipeline.create<dai::node::SystemLogger>();
  sysLog->setRate(0.2f);
  auto sysLogQueue = sysLog->out.createOutputQueue();

  pipeline.start();

  // ── CameraInfo: intrinsics at the actual streamed resolution ─────────────
  // DepthAI stores the factory calibration in EEPROM. getCameraIntrinsics()
  // rescales it for the requested destShape with the same center-crop / aspect
  // handling the Camera node applied to the NV12 output, so the K we publish
  // matches the pixels the receiver decodes — whatever the streamed size.
  dai::CalibrationHandler calib;
  try {
    calib = device->readCalibration();
  } catch (const std::exception &e) {
    RCLCPP_WARN(node->get_logger(), "No EEPROM calibration (%s) — using defaults", e.what());
    calib = device->readCalibrationOrDefault();
  }

  std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info;
  std::mutex camera_info_mtx;
  auto camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/oakd/camera_info", rclcpp::QoS(1).transient_local().reliable());

  // AprilTag + stream telemetry, single std_msgs/String, published once per
  // detection pass (~5 Hz): "oakd_fps=.. apriltag_fps=.. avg_margin=.. num_tags=.."
  // Best-effort, depth 1: only the latest value ever matters, so no reason to
  // pay RELIABLE's ACK/retry latency for a value that's superseded in ~200ms
  // anyway -- matches the receiver's subscriber QoS below (must agree,
  // RELIABLE can't receive from a BEST_EFFORT publisher).
  auto telemetry_pub = node->create_publisher<std_msgs::msg::String>(
      "/oakd/apriltag_telemetry", rclcpp::QoS(1).best_effort());

  // Marker board pose in base_link: T_baselink_board = T_baselink_camera *
  // T_camera_board. T_camera_board comes from this node's own solvePnP
  // (below); T_baselink_camera is head_camera_tf_bridge's already-resolved
  // head_camera_frame -> base_link transform (FK + URDF mount offset), which
  // this node just subscribes to and caches -- no TF listener needed here.
  auto board_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/oakd/marker_board_pose", rclcpp::QoS(1).best_effort());
  geometry_msgs::msg::TransformStamped latest_cam_tf;
  bool have_cam_tf = false;
  auto cam_tf_sub = node->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/head_camera_tf", rclcpp::QoS(1).transient_local().reliable(),
      [&](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        latest_cam_tf = *msg;
        have_cam_tf = true;
      });
  const std::map<int, std::array<cv::Point3f, 4>> board_corners = buildBoardCorners();
  cv::Mat pnp_rvec, pnp_tvec;
  bool pnp_seeded = false;
  constexpr double kCamTfMaxAgeS = 0.5;  // ignore a stale head_camera_tf

  // Build a CameraInfo for the given output size from the factory calibration.
  auto build_camera_info = [&](uint32_t w, uint32_t h) {
    auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    msg->header.frame_id = "head_camera_frame";
    msg->header.stamp = node->now();
    msg->width  = w;
    msg->height = h;
    msg->distortion_model = "rational_polynomial";

    auto K = calib.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, dai::Size2f(w, h));
    if (K.size() == 3 && K[0].size() == 3) {
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          msg->k[r * 3 + c] = K[r][c];
      // p = K | 0 (no rectification offset — the optical center is the image center)
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) msg->p[r * 4 + c] = msg->k[r * 3 + c];
        msg->p[r * 4 + 3] = 0.0f;
      }
    }
    // r = identity: the pipeline applies no rectification rotation.
    msg->r[0] = msg->r[4] = msg->r[8] = 1.0f;

    // DepthAI stores OpenCV rational-polynomial coefficients (perspective model),
    // ordered [k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,taux,tauy]. ROS
    // distortion_model="rational_polynomial" wants exactly the first 8; that
    // reduces to plumb_bob when k4..k6 are zero, so this is correct either way.
    auto dist = calib.getDistortionCoefficients(dai::CameraBoardSocket::CAM_A);
    if (dist.size() >= 8) msg->d.assign(dist.begin(), dist.begin() + 8);
    return msg;
  };

  // Republish on a slow timer so late-joining subscribers get the current
  // intrinsics (transient_local keeps the last message for brand-new joins;
  // the timer just refreshes the stamp for liveness checks).
  auto camera_info_timer = node->create_wall_timer(std::chrono::milliseconds(1000), [&]() {
    std::lock_guard<std::mutex> lk(camera_info_mtx);
    if (camera_info) {
      camera_info->header.stamp = node->now();
      camera_info_pub->publish(*camera_info);
    }
  });

  // ── Telemetry UDP Socket ─────────────────────────────────────────────────
  int telemetry_sock = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in telemetry_addr;
  memset(&telemetry_addr, 0, sizeof(telemetry_addr));
  telemetry_addr.sin_family = AF_INET;
  telemetry_addr.sin_port = htons(telemetry_port);
  inet_pton(AF_INET, dest_ip.c_str(), &telemetry_addr.sin_addr);

  // ── RTT Calibration: bind telemetry socket to receive CAL_REQ ──────────
  struct sockaddr_in local_telem_addr;
  memset(&local_telem_addr, 0, sizeof(local_telem_addr));
  local_telem_addr.sin_family = AF_INET;
  local_telem_addr.sin_port = htons(telemetry_port);
  local_telem_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(telemetry_sock, (struct sockaddr*)&local_telem_addr,
           sizeof(local_telem_addr)) < 0) {
    RCLCPP_WARN(node->get_logger(), "Calibration bind failed — no RTT handshake");
  }

  // Receive timeout so the cal thread can check rclcpp::ok()
  struct timeval cal_tv;
  cal_tv.tv_sec = 1;
  cal_tv.tv_usec = 0;
  setsockopt(telemetry_sock, SOL_SOCKET, SO_RCVTIMEO, &cal_tv, sizeof(cal_tv));

  // Latest host clock stamp, shared with the calibration thread
  std::atomic<double> latest_host_ms{-1.0};

  // Calibration listener thread (non-blocking, detached)
  std::thread cal_thread([&]() {
    char buf[256];
    struct sockaddr_in from_addr;
    socklen_t from_len;
    while (rclcpp::ok()) {
      from_len = sizeof(from_addr);
      int n = recvfrom(telemetry_sock, buf, sizeof(buf) - 1, 0,
                       (struct sockaddr*)&from_addr, &from_len);
      if (n > 0) {
        buf[n] = '\0';
        if (strncmp(buf, "CAL_REQ", 7) == 0) {
          double t = latest_host_ms.load();
          if (t >= 0.0) {
            char resp[256];
            int rn = snprintf(resp, sizeof(resp), "CAL_RES oakt=%.3f", t);
            sendto(telemetry_sock, resp, rn, 0,
                   (struct sockaddr*)&from_addr, sizeof(from_addr));
          }
        }
      }
    }
  });
  cal_thread.detach();

  // ── GStreamer Pipeline (host-CPU x264) ───────────────────────────────────
  //   - videoconvert(NV12->I420) then x264enc, mirroring realsense_udp_streamer
  //   - ultrafast + zerolatency: lowest-latency x264 preset (matches the D405
  //     encoder in realsense_udp_streamer.cpp). Was veryfast (better
  //     compression at the same 20 Mbps budget, at the cost of a bit more
  //     per-frame encode time) -- switched to prioritize latency; expect
  //     slightly worse image quality at the same bitrate as the tradeoff.
  //   - config-interval=-1: h264parse resends SPS/PPS before every IDR →
  //     receiver can recover from late join / packet loss within one GOP
  //   - block=false: if the host encoder can't keep up, drop frames not stall
  std::string gst_pipeline_str =
    "appsrc name=src is-live=true format=3 do-timestamp=false block=false "
    "caps=\"video/x-raw,format=NV12,width=" + std::to_string(k_out_width) +
    ",height=" + std::to_string(k_out_height) + ",framerate=" +
    std::to_string(fps) + "/1\" ! "
    "videoconvert ! video/x-raw,format=I420 ! "
    "x264enc speed-preset=ultrafast tune=zerolatency bitrate=" +
    std::to_string(bitrate_kbps) + " key-int-max=" + std::to_string(fps) + " ! "
    "h264parse config-interval=-1 ! "
    "rtph264pay pt=96 ! "
    "udpsink host=" + dest_ip + " port=" + std::to_string(video_port) +
    " sync=false async=false";

  GError *error = nullptr;
  GstElement *gst_pipeline = gst_parse_launch(gst_pipeline_str.c_str(), &error);
  if (error) {
    RCLCPP_ERROR(node->get_logger(), "GStreamer error: %s", error->message);
    g_error_free(error);
    rclcpp::shutdown();
    return -1;
  }

  GstElement *appsrc = gst_bin_get_by_name(GST_BIN(gst_pipeline), "src");
  gst_element_set_state(gst_pipeline, GST_STATE_PLAYING);

  RCLCPP_INFO(node->get_logger(), "Streaming raw 720p -> host x264 over UDP to %s:%d, telemetry -> %s:%d",
              dest_ip.c_str(), video_port, dest_ip.c_str(), telemetry_port);

  // ── AprilTag detector (25h9) ─────────────────────────────────────────────
  // Reuses the SAME raw frame the encoder gets below -- no second Camera
  // output, no timing mismatch. Rate-limited to ~5 Hz via a wall-clock gate
  // (see kDetectPeriodS below), not a frame-count modulo -- correct
  // regardless of what the camera actually delivers vs the requested fps.
  apriltag_family_t *tag_family = tag25h9_create();
  apriltag_detector_t *tag_detector = apriltag_detector_create();
  apriltag_detector_add_family(tag_detector, tag_family);
  constexpr double kDetectPeriodS = 0.2;  // 5 Hz
  auto last_detect_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  RCLCPP_INFO(node->get_logger(), "[AprilTag] 25h9 detector ready, ~%.1f Hz (wall-clock gated)",
              1.0 / kDetectPeriodS);

  // Telemetry state shared between the 5s OAK-D fps report below and the
  // per-detection-pass /oakd/apriltag_telemetry publish.
  double current_oakd_fps = 0.0;
  int apriltag_pass_count = 0;
  double current_apriltag_fps = 0.0;
  auto last_apriltag_fps_time = std::chrono::steady_clock::now();

  // ── Frame loop ───────────────────────────────────────────────────────────
  int frame_count = 0;
  int last_reported_count = 0;
  auto last_report_time = std::chrono::steady_clock::now();
  double last_frame_ts = -1.0;
  double worst_delay_ms = 0.0;

  // Monotonic PTS baseline: first OAK-D hardware timestamp
  double hw_base_ms = -1.0;

  while (rclcpp::ok() && pipeline.isRunning()) {
    bool hasTimedOut = false;
    auto videoFrame = videoQueue->get<dai::ImgFrame>(std::chrono::milliseconds(500), hasTimedOut);

    if (videoFrame && !hasTimedOut) {
      const auto& data = videoFrame->getData();

      // First frame: the actual streamed size is authoritative (the Camera node
      // may crop/round the requested output). Publish the CameraInfo for it once.
      {
        uint32_t actual_w = videoFrame->getWidth();
        uint32_t actual_h = videoFrame->getHeight();
        std::lock_guard<std::mutex> lk(camera_info_mtx);
        if (!camera_info) {
          camera_info = build_camera_info(actual_w, actual_h);
          camera_info_pub->publish(*camera_info);
          RCLCPP_INFO(node->get_logger(),
                      "Published intrinsics on /oakd/camera_info: %ux%u "
                      "fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                      camera_info->width, camera_info->height,
                      camera_info->k[0], camera_info->k[4],
                      camera_info->k[2], camera_info->k[5]);
        }
      }

      GstBuffer *buffer = gst_buffer_new_allocate(nullptr, data.size(), nullptr);
      GstMapInfo map;
      if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        memcpy(map.data, data.data(), data.size());
        gst_buffer_unmap(buffer, &map);
      }

      // Set monotonic PTS from OAK-D hardware timestamp so GStreamer has a
      // proper, strictly-increasing timeline (same fix as the 1080p fallback).
      double hw_ms = videoFrame->getTimestampDevice().time_since_epoch().count() / 1e6;
      if (hw_base_ms < 0.0) {
        hw_base_ms = hw_ms;
      }
      double elapsed_ms = hw_ms - hw_base_ms;
      GstClockTime pts = (GstClockTime)(elapsed_ms * 1e6); // ms -> ns
      GST_BUFFER_PTS(buffer) = pts;
      GST_BUFFER_DTS(buffer) = pts;
      GST_BUFFER_DURATION(buffer) = GST_SECOND / fps;

      GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
      if (ret != GST_FLOW_OK) {
        RCLCPP_WARN(node->get_logger(), "Failed to push buffer (ret=%d)", ret);
      }

      frame_count++;

      // ── AprilTag detection (~5 Hz, wall-clock gated -- see above) ──────────
      // NV12's Y-plane (luma) is the first width*height bytes of the buffer
      // and IS an 8-bit grayscale image already -- no colorspace conversion
      // needed. Copied into apriltag's own aligned image_u8_t (required for
      // its internal SIMD code) rather than aliased, since `data` is about to
      // be handed to GStreamer above/below and must not be mutated.
      auto detect_now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(detect_now - last_detect_time).count() >= kDetectPeriodS) {
        last_detect_time = detect_now;
        uint32_t dw = videoFrame->getWidth();
        uint32_t dh = videoFrame->getHeight();
        if (data.size() >= static_cast<size_t>(dw) * dh) {
          image_u8_t *im = image_u8_create(dw, dh);
          for (uint32_t row = 0; row < dh; ++row) {
            memcpy(im->buf + row * im->stride, data.data() + row * dw, dw);
          }
          // No console log here -- /oakd/apriltag_telemetry (published below)
          // is the only output; the receiver subscribes and draws it on the
          // dashboard overlay instead of this process spamming stdout.
          zarray_t *detections = apriltag_detector_detect(tag_detector, im);
          int n = zarray_size(detections);
          double margin_sum = 0.0;
          // Combined-corners bundle: every detected tag that's also in the
          // board layout contributes its 4 corners to ONE solvePnP call
          // below, rather than averaging independent per-tag poses.
          std::vector<cv::Point3f> obj_pts;
          std::vector<cv::Point2f> img_pts;
          for (int i = 0; i < n; ++i) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            margin_sum += det->decision_margin;
            auto it = board_corners.find(det->id);
            if (it != board_corners.end()) {
              for (int k = 0; k < 4; ++k) {
                obj_pts.push_back(it->second[k]);
                img_pts.push_back(cv::Point2f(static_cast<float>(det->p[k][0]),
                                              static_cast<float>(det->p[k][1])));
              }
            }
          }
          double avg_margin = (n > 0) ? margin_sum / n : 0.0;
          apriltag_detections_destroy(detections);
          image_u8_destroy(im);

          // Board pose: solve once over every matched tag's corners, seeded
          // from the previous frame's pose (cheap: consecutive frames barely
          // move), then compose with the cached head_camera->base_link
          // transform to publish the board's pose in base_link.
          if (obj_pts.size() >= 4 && camera_info) {
            cv::Mat K = (cv::Mat_<double>(3, 3) <<
              camera_info->k[0], camera_info->k[1], camera_info->k[2],
              camera_info->k[3], camera_info->k[4], camera_info->k[5],
              camera_info->k[6], camera_info->k[7], camera_info->k[8]);
            cv::Mat D(1, static_cast<int>(camera_info->d.size()), CV_64F);
            for (size_t i = 0; i < camera_info->d.size(); ++i) {
              D.at<double>(0, static_cast<int>(i)) = camera_info->d[i];
            }
            bool pnp_ok = cv::solvePnP(obj_pts, img_pts, K, D, pnp_rvec, pnp_tvec,
                                       pnp_seeded, cv::SOLVEPNP_ITERATIVE);
            if (pnp_ok) {
              pnp_seeded = true;
              double cam_tf_age = have_cam_tf
                  ? (node->now() - rclcpp::Time(latest_cam_tf.header.stamp)).seconds()
                  : 1e9;
              if (have_cam_tf && cam_tf_age <= kCamTfMaxAgeS) {
                Eigen::Vector3d rv(pnp_rvec.at<double>(0), pnp_rvec.at<double>(1),
                                   pnp_rvec.at<double>(2));
                double ang = rv.norm();
                Eigen::Matrix3d R_cam_board = Eigen::Matrix3d::Identity();
                if (ang > 1e-12) {
                  R_cam_board = Eigen::AngleAxisd(ang, rv / ang).toRotationMatrix();
                }
                Eigen::Vector3d t_cam_board(pnp_tvec.at<double>(0), pnp_tvec.at<double>(1),
                                            pnp_tvec.at<double>(2));

                const auto &ct = latest_cam_tf.transform;
                Eigen::Quaterniond q_base_cam(ct.rotation.w, ct.rotation.x,
                                              ct.rotation.y, ct.rotation.z);
                Eigen::Matrix3d R_base_cam = q_base_cam.toRotationMatrix();
                Eigen::Vector3d t_base_cam(ct.translation.x, ct.translation.y,
                                           ct.translation.z);

                Eigen::Matrix3d R_base_board = R_base_cam * R_cam_board;
                Eigen::Vector3d t_base_board = R_base_cam * t_cam_board + t_base_cam;
                Eigen::Quaterniond q_base_board(R_base_board);

                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = node->now();
                pose_msg.header.frame_id = "base_link";
                pose_msg.pose.position.x = t_base_board.x();
                pose_msg.pose.position.y = t_base_board.y();
                pose_msg.pose.position.z = t_base_board.z();
                pose_msg.pose.orientation.w = q_base_board.w();
                pose_msg.pose.orientation.x = q_base_board.x();
                pose_msg.pose.orientation.y = q_base_board.y();
                pose_msg.pose.orientation.z = q_base_board.z();
                board_pose_pub->publish(pose_msg);
              }
            }
          }

          // Rolling apriltag_fps: passes/second over a 1s window.
          apriltag_pass_count++;
          double fps_elapsed = std::chrono::duration<double>(detect_now - last_apriltag_fps_time).count();
          if (fps_elapsed >= 1.0) {
            current_apriltag_fps = apriltag_pass_count / fps_elapsed;
            apriltag_pass_count = 0;
            last_apriltag_fps_time = detect_now;
          }

          std_msgs::msg::String telemetry_msg;
          std::ostringstream tel;
          tel << "oakd_fps=" << std::fixed << std::setprecision(1) << current_oakd_fps
              << " apriltag_fps=" << std::fixed << std::setprecision(1) << current_apriltag_fps
              << " avg_margin=" << std::fixed << std::setprecision(1) << avg_margin
              << " num_tags=" << n;
          telemetry_msg.data = tel.str();
          telemetry_pub->publish(telemetry_msg);
        }
      }

      // Sample sender host time for latency measurement (steady_clock, ms since boot)
      auto send_now = std::chrono::steady_clock::now();
      double send_host_ms = std::chrono::duration<double, std::milli>(
          send_now.time_since_epoch()).count();
      latest_host_ms.store(send_host_ms);

      // Telemetry: track worst inter-frame gap
      double hw_ts = videoFrame->getTimestampDevice().time_since_epoch().count() / 1e6;
      if (last_frame_ts >= 0.0) {
        double gap = hw_ts - last_frame_ts;
        if (gap > worst_delay_ms) worst_delay_ms = gap;
      }
      last_frame_ts = hw_ts;

      double elapsed = std::chrono::duration<double>(send_now - last_report_time).count();
      if (elapsed >= 5.0) {
        int delta = frame_count - last_reported_count;
        double fps_val = delta / elapsed;
        current_oakd_fps = fps_val;
        std::ostringstream ss;
        ss << "[OAK-720p] FPS: " << std::fixed << std::setprecision(1) << fps_val
           << " | Worst Delay: " << std::fixed << std::setprecision(1) << worst_delay_ms << " ms"
           << " | Codec: H264-CPU-x264@" << bitrate_kbps / 1000 << "Mbps"
           << " | TW:" << std::fixed << std::setprecision(1) << send_host_ms;
        sendUdpText(telemetry_sock, telemetry_addr, ss.str());
        last_reported_count = frame_count;
        last_report_time = send_now;
        worst_delay_ms = 0.0;
      }
    }

    auto sysInfo = sysLogQueue->tryGet<dai::SystemInformation>();
    if (sysInfo) {
      printSystemInformation(*sysInfo, telemetry_sock, telemetry_addr);
    }

    rclcpp::spin_some(node);
  }

  sendUdpText(telemetry_sock, telemetry_addr, "[OAK-720p] Shutting down");
  if (telemetry_sock >= 0) close(telemetry_sock);
  gst_element_set_state(gst_pipeline, GST_STATE_NULL);
  gst_object_unref(appsrc);
  gst_object_unref(gst_pipeline);
  apriltag_detector_destroy(tag_detector);
  tag25h9_destroy(tag_family);

  rclcpp::shutdown();
  return 0;
}
