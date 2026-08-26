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
#include <iomanip>
#include <iostream>
#include <sstream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <memory>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <cstdlib>

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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("depthai_720p_raw_udp_node");

  // ── Positional args (fall back to defaults if not supplied) ─────────────
  const std::string dest_ip   = (argc > 1) ? argv[1] : "192.168.0.241";
  const int  video_port       = (argc > 2) ? std::atoi(argv[2]) : 9110;
  const int  fps              = (argc > 3) ? std::atoi(argv[3]) : 30;
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
              "USB Speed: %d  |  codec: host-CPU x264enc (veryfast/zerolatency) @ %d kbps | %dx%d@%d",
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
  auto *videoOut = cam->requestOutput({k_out_width, k_out_height}, dai::ImgFrame::Type::NV12);

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
  //   - veryfast + zerolatency = still no B-frames / no lookahead (low latency),
  //     but far better compression than ultrafast → the 20 Mbps budget is used
  //   - config-interval=-1: h264parse resends SPS/PPS before every IDR →
  //     receiver can recover from late join / packet loss within one GOP
  //   - block=false: if the host encoder can't keep up, drop frames not stall
  std::string gst_pipeline_str =
    "appsrc name=src is-live=true format=3 do-timestamp=false block=false "
    "caps=\"video/x-raw,format=NV12,width=" + std::to_string(k_out_width) +
    ",height=" + std::to_string(k_out_height) + ",framerate=" +
    std::to_string(fps) + "/1\" ! "
    "videoconvert ! video/x-raw,format=I420 ! "
    "x264enc speed-preset=veryfast tune=zerolatency bitrate=" +
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

  rclcpp::shutdown();
  return 0;
}
