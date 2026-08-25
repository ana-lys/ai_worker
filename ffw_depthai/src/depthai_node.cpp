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
  ss << "[OAK-D] "
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
  auto node = std::make_shared<rclcpp::Node>("depthai_mjpeg_udp_node");

  // ── ROS parameter: select codec ──────────────────────────────────────────
  node->declare_parameter<bool>("use_h264", false);
  const bool use_h264 = node->get_parameter("use_h264").as_bool();

  gst_init(nullptr, nullptr);

  std::shared_ptr<dai::Device> device;
  try {
    device = std::make_shared<dai::Device>();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect to device: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "USB Speed: %d  |  codec: %s",
              static_cast<int>(device->getUsbSpeed()),
              use_h264 ? "H264-Baseline" : "MJPEG");

  dai::Pipeline pipeline(device);

  auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
  cam->initialControl.setSharpness(6);     // 6/8 — crisper edges, compensates for encode softness
  cam->initialControl.setLumaDenoise(1);   // minimal luma denoise, preserves fine detail
  cam->initialControl.setChromaDenoise(1); // mild chroma denoise (was 3 — was blurring colored edges)

  auto *videoOut = cam->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12);

  auto videoEnc = pipeline.create<dai::node::VideoEncoder>();

  if (use_h264) {
    // ── H264 Baseline: no B-frames → strictly linear PTS order ───────────
    videoEnc->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H264_BASELINE);
    videoEnc->setNumBFrames(0);           // explicitly disable B-frames (safety)
    videoEnc->setKeyframeFrequency(30);   // IDR every 1s → fast recovery on packet loss
    videoEnc->setBitrateKbps(8000);        // ~8 Mbps for 1080p30 — CBR keeps encoder at steady 30fps
    videoEnc->setRateControlMode(dai::VideoEncoderProperties::RateControlMode::CBR);
  } else {
    // ── MJPEG fallback (original settings) ───────────────────────────────
    videoEnc->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::MJPEG);
    videoEnc->setQuality(90);
  }

  videoOut->link(videoEnc->input);

  // Queue depth = 1, blocking = false → always drop oldest, never accumulate latency
  auto videoQueue = videoEnc->bitstream.createOutputQueue(1, false);

  auto sysLog = pipeline.create<dai::node::SystemLogger>();
  sysLog->setRate(0.2f);
  auto sysLogQueue = sysLog->out.createOutputQueue();

  pipeline.start();

  // ── CameraInfo: intrinsics at the actual streamed resolution ─────────────
  // DepthAI stores the factory calibration in EEPROM. getCameraIntrinsics()
  // rescales it for the requested destShape with the same center-crop / aspect
  // handling the Camera node applied to the frame (1920x1080 from the 4:3
  // IMX378 sensor → 16:9 CROP model), so the K we publish matches the pixels
  // the receiver decodes.
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
    msg->header.frame_id = "oakd_cam_a";
    msg->header.stamp = node->now();
    msg->width  = w;
    msg->height = h;
    msg->distortion_model = "plumb_bob";

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

    // DepthAI order [k1,k2,p1,p2,k3,...]; ROS plumb_bob wants exactly [k1,k2,p1,p2,k3].
    auto dist = calib.getDistortionCoefficients(dai::CameraBoardSocket::CAM_A);
    if (dist.size() >= 5) msg->d.assign(dist.begin(), dist.begin() + 5);
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
  const std::string dest_ip = "192.168.0.241";
  const int video_port = 9100;
  const int telemetry_port = video_port + 200; // 9300

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

  // Latest OAK-D HW timestamp, shared with the calibration thread
  std::atomic<double> latest_oakd_hw_ms{-1.0};

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
          double oakt = latest_oakd_hw_ms.load();
          if (oakt >= 0.0) {
            char resp[256];
            int rn = snprintf(resp, sizeof(resp),
                              "CAL_RES oakt=%.3f", oakt);
            sendto(telemetry_sock, resp, rn, 0,
                   (struct sockaddr*)&from_addr, sizeof(from_addr));
          }
        }
      }
    }
  });
  cal_thread.detach();

  // ── GStreamer Pipeline ───────────────────────────────────────────────────
  GError *error = nullptr;
  std::string gst_pipeline_str;

  if (use_h264) {
    // H264 RTP sender:
    //   - video/x-h264 caps tell GStreamer the format before h264parse
    //   - stream-format=byte-stream + alignment=au = one complete AU per buffer
    //   - h264parse reframes NALs correctly for RTP payloading
    //   - config-interval=1 sends SPS/PPS with every keyframe → receiver can
    //     recover from late join or packet loss every IDR period (1s)
    //   - block=false: if GStreamer can't keep up, we drop frames rather than stall
    gst_pipeline_str =
      "appsrc name=src is-live=true format=3 do-timestamp=false block=false "
      "caps=\"video/x-h264,stream-format=byte-stream,alignment=au,"
      "framerate=30/1,width=1920,height=1080,profile=baseline\" ! "
      "h264parse ! "
      "rtph264pay config-interval=1 pt=96 ! "
      "udpsink host=192.168.0.241 port=9100 sync=false async=false";
  } else {
    // MJPEG sender (unchanged)
    gst_pipeline_str =
      "appsrc name=src is-live=true format=3 do-timestamp=true block=false ! "
      "image/jpeg,framerate=30/1,width=1920,height=1080 ! "
      "rtpjpegpay ! "
      "udpsink host=192.168.0.241 port=9100 sync=false async=false";
  }

  GstElement *gst_pipeline = gst_parse_launch(gst_pipeline_str.c_str(), &error);
  if (error) {
    RCLCPP_ERROR(node->get_logger(), "GStreamer error: %s", error->message);
    g_error_free(error);
    rclcpp::shutdown();
    return -1;
  }

  GstElement *appsrc = gst_bin_get_by_name(GST_BIN(gst_pipeline), "src");
  gst_element_set_state(gst_pipeline, GST_STATE_PLAYING);

  RCLCPP_INFO(node->get_logger(), "Streaming %s over UDP to %s:%d, telemetry -> %s:%d",
              use_h264 ? "H264-Baseline" : "MJPEG",
              dest_ip.c_str(), video_port, dest_ip.c_str(), telemetry_port);

  // ── Frame loop ───────────────────────────────────────────────────────────
  int frame_count = 0;
  int last_reported_count = 0;
  auto last_report_time = std::chrono::steady_clock::now();
  double last_frame_ts = -1.0;
  double worst_delay_ms = 0.0;

  // For H264 monotonic PTS: track first OAK-D hardware timestamp as baseline
  double       hw_base_ms   = -1.0;

  while (rclcpp::ok() && pipeline.isRunning()) {
    bool hasTimedOut = false;
    auto videoFrame = videoQueue->get<dai::ImgFrame>(std::chrono::milliseconds(500), hasTimedOut);

    if (videoFrame && !hasTimedOut) {
      const auto& data = videoFrame->getData();

      // First frame: publish the CameraInfo for the streamed resolution once.
      // The encoded ImgFrame still carries the source frame dimensions; fall
      // back to the requested 1920x1080 if the metadata is unset.
      {
        uint32_t actual_w = videoFrame->getWidth();
        uint32_t actual_h = videoFrame->getHeight();
        if (actual_w == 0 || actual_h == 0) { actual_w = 1920; actual_h = 1080; }
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

      if (use_h264) {
        // Set monotonic PTS from OAK-D hardware timestamp so GStreamer has a
        // proper, strictly-increasing timeline. This is the key fix that
        // prevents the decoder from getting confused about frame order.
        double hw_ms = videoFrame->getTimestampDevice().time_since_epoch().count() / 1e6;
        if (hw_base_ms < 0.0) {
          hw_base_ms = hw_ms;
        }
        double elapsed_ms = hw_ms - hw_base_ms;
        GstClockTime pts = (GstClockTime)(elapsed_ms * 1e6); // ms -> ns
        GST_BUFFER_PTS(buffer) = pts;
        GST_BUFFER_DTS(buffer) = pts;
        GST_BUFFER_DURATION(buffer) = GST_SECOND / 30; // 1/30s in ns
      } else {
        // MJPEG: let do-timestamp=true handle it
        GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
      }

      GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
      if (ret != GST_FLOW_OK) {
        RCLCPP_WARN(node->get_logger(), "Failed to push buffer (ret=%d)", ret);
      }

      frame_count++;

      // Sample sender host time for latency measurement (steady_clock, ms since boot)
      auto send_now = std::chrono::steady_clock::now();
      double send_host_ms = std::chrono::duration<double, std::milli>(
          send_now.time_since_epoch()).count();
      latest_oakd_hw_ms.store(send_host_ms);

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
        ss << "[OAK-D] FPS: " << std::fixed << std::setprecision(1) << fps_val
           << " | Worst Delay: " << std::fixed << std::setprecision(1) << worst_delay_ms << " ms"
           << " | Codec: " << (use_h264 ? "H264-Baseline" : "MJPEG")
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

  sendUdpText(telemetry_sock, telemetry_addr, "[OAK-D] Shutting down");
  if (telemetry_sock >= 0) close(telemetry_sock);
  gst_element_set_state(gst_pipeline, GST_STATE_NULL);
  gst_object_unref(appsrc);
  gst_object_unref(gst_pipeline);

  rclcpp::shutdown();
  return 0;
}
