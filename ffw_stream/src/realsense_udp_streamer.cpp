// udp_depth_ir_streamer.cpp
//
// For each connected RealSense camera:
//   - Opens Depth + IR(1) streams (D405) or RGB-only (D435).
//   - Clamps depth to [0, max_depth_m] meters and rescales to 8-bit (0-255).
//   - IR frames are already 8-bit (Y8), copied out respecting row stride.
//   - Encodes via GStreamer appsrc → x264enc (software libx264, ultrafast +
//     zerolatency) → rtph264pay → udpsink. In-process, no subprocess overhead.
//
// NOTE on depth=0: the RealSense SDK uses raw value 0 to mean "no valid
// depth" (sensor couldn't measure that pixel). With this mapping that also
// comes out as 0 in the 8-bit image -- visually identical to "actual
// distance = 0m". If you need to tell those apart downstream, special-case
// raw==0 before scaling (e.g. force it to 255) and document the convention
// for whoever consumes the stream.
//
// Usage:
//   realsense_udp_streamer <dest_ip> <base_port> [width=480] [height=270]
//   [fps=30] [max_depth_m=1.0] [--enable-d405s|--disable-d405s]
//   [--d435-rgb|--no-d435-rgb] [--dual-rgb-no-depth] [--disable-left-d405]
//   [--color-exposure <microseconds>] [--d435-fps <n>]
//
// The head D435/D435i is identified by USB product ID (or model name), not
// serial, so any unit works; it always streams 1280x720 RGB at --d435-fps
// (default: the shared fps; the D435 color sensor accepts 6/15/30).
//
// Port mapping (per camera index i, 0-based):
//   depth -> base_port + i*2   (unused when --dual-rgb-no-depth)
//   ir    -> base_port + i*2 + 1   (RGB for both D405s under --dual-rgb-no-depth)
//
// Build: colcon build --packages-select ffw_stream

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <iomanip>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <condition_variable>
#include <map>
#include <memory>

// GStreamer appsrc-based H264 encoding
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// ROS node so this executable can publish the D435 intrinsics with zero-latency
// (transient_local) QoS — same pattern as the OAK-D streamer's /oakd/camera_info.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// AprilTag 25h9 board-pose tap on the D435 head stream (own thread).
#include "ffw_stream/board_pose_detector.hpp"

std::mutex cout_mutex;
std::atomic<bool> g_running{true};

// Monitor variables
// Keys are inserted in main() BEFORE the camera threads start; threads only
// touch existing entries (concurrent std::map insertion is a data race).
std::map<std::string, std::atomic<int>> frames_captured;
// Camera bring-up (pipe.start + option writes + restart) is serialized: with
// three RealSense devices initializing in parallel, librealsense's RS-USB
// backend deadlocked on its global USB lock (lock_singleton) with a control
// transfer hung -- every camera thread blocked forever, nothing streamed and
// nothing was logged (2026-10-08, D435 + 2x D405).
std::mutex g_rs_init_mutex;
std::mutex timestamp_mutex;
std::map<std::string, double> latest_timestamps;
bool phase_delta_printed = false;

void log(const std::string &msg) {
  std::lock_guard<std::mutex> lock(cout_mutex);
  std::cout << msg << std::endl;
}

void on_sigint(int) { g_running = false; }

// ── GStreamer appsrc-based H264 encoding ────────────────────────────

struct GstEncoder {
  GstElement *pipeline = nullptr;
  GstElement *appsrc    = nullptr;
  uint64_t    pts_counter = 0;
  bool        mjpeg     = false;   // MJPEG pipeline → stamp real ns PTS for jpegenc
  uint64_t    frame_interval_ns = 0;
};

// Build an appsrc-based encoding + UDP streaming pipeline.
// rgb_mode=true → RGB24 input; false → GRAY8 input.
// mjpeg=true → jpegenc q90 (intra-only zero-latency); false → x264enc
// ultrafast/zerolatency H264. Shared videoconvert→I420 stage feeds both.
// bitrate_kbps > 0 sets x264's target; 0 leaves x264's 2048 kbps default
// (fine for the 480x270 D405 streams, far too low for 720p -- it is what made
// the D435 head stream blocky).
GstEncoder create_gst_stream(const std::string &ip, int port, int width, int height,
                            int fps, bool rgb_mode, bool mjpeg, int bitrate_kbps = 0) {
  GstEncoder enc;
  enc.mjpeg = mjpeg;
  enc.frame_interval_ns = GST_SECOND / fps;
  std::string fmt = rgb_mode ? "RGB" : "GRAY8";
  std::ostringstream caps;
  caps << "video/x-raw,format=" << fmt
       << ",width=" << width << ",height=" << height
       << ",framerate=" << fps << "/1";

  std::ostringstream pipe;
  pipe << "appsrc name=src is-live=true format=3 do-timestamp=false block=false "
       << "caps=\"" << caps.str() << "\" ! "
       << "videoconvert ! "
       << "video/x-raw,format=I420 ! ";
  if (mjpeg) {
    // Intra-only frames → no GOP/reorder delay, instant decode, per-frame loss recovery
    pipe << "jpegenc quality=90 ! "
         << "rtpjpegpay ! "
         << "udpsink host=" << ip << " port=" << port << " sync=false async=false";
  } else {
    pipe << "x264enc speed-preset=ultrafast tune=zerolatency key-int-max=" << fps;
    if (bitrate_kbps > 0) pipe << " bitrate=" << bitrate_kbps;
    // buffer-size: ask for a 4 MB socket send buffer so a keyframe burst is not
    // dropped at the socket (Udp SndbufErrors). The kernel clamps it to
    // net.core.wmem_max (208 KB stock) -- raise that on the robot to benefit.
    pipe << " ! h264parse config-interval=-1 ! "
         << "rtph264pay pt=96 ! "
         << "udpsink host=" << ip << " port=" << port
         << " sync=false async=false buffer-size=4194304";
  }

  GError *error = nullptr;
  enc.pipeline = gst_parse_launch(pipe.str().c_str(), &error);
  if (error) {
    log("GStreamer error: " + std::string(error->message));
    g_error_free(error);
    return enc;
  }

  enc.appsrc = gst_bin_get_by_name(GST_BIN(enc.pipeline), "src");
  gst_element_set_state(enc.pipeline, GST_STATE_PLAYING);
  return enc;
}

// Push a raw frame (GRAY8 or RGB24) into a GstEncoder pipeline.
bool gst_encoder_push_frame(GstEncoder &enc, const uint8_t *data, size_t size) {
  if (!enc.pipeline || !enc.appsrc) return false;

  GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    std::memcpy(map.data, data, size);
    gst_buffer_unmap(buffer, &map);
  }

  if (enc.mjpeg) {
    // Proper monotonic PTS in ns. jpegenc/rtpjpegpay stamp the RTP timeline from the
    // 90 kHz clock; a raw frame-index counter (µs-scale "ns") would collapse to ~0.
    // H264 path keeps the frame-index behavior exactly as before.
    GST_BUFFER_PTS(buffer)      = enc.pts_counter * enc.frame_interval_ns;
    enc.pts_counter++;
    GST_BUFFER_DURATION(buffer) = enc.frame_interval_ns;
  } else {
    GST_BUFFER_PTS(buffer)      = enc.pts_counter++;
    GST_BUFFER_DURATION(buffer) = GST_SECOND / 30;
  }

  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(enc.appsrc), buffer);
  return ret == GST_FLOW_OK;
}

// Stop and free a GstEncoder pipeline.
void destroy_gst_stream(GstEncoder &enc) {
  if (enc.pipeline) {
    gst_element_set_state(enc.pipeline, GST_STATE_NULL);
    gst_object_unref(enc.appsrc);
    gst_object_unref(enc.pipeline);
    enc.pipeline = nullptr;
    enc.appsrc   = nullptr;
  }
}

void stream_camera_rgb(const std::string &serial, const std::string &dest_ip, int port,
                       int width, int height, int fps, bool mjpeg,
                       rclcpp::Node::SharedPtr node,
                       rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub,
                       bool apriltag, int bitrate_kbps) {
  std::ostringstream hdr;
  hdr << "\n=== CAM (RGB ZED-Replacement) " << serial << " : rgb->udp:" << port
      << "  gst=" << (mjpeg ? "mjpeg q90" : "h264 @ " + std::to_string(bitrate_kbps) + " kbps")
      << " ===";
  log(hdr.str());

  GstEncoder gst_enc = create_gst_stream(dest_ip, port, width, height, fps, true, mjpeg,
                                         bitrate_kbps);

  if (!gst_enc.pipeline) {
    log("CAM RGB " + serial + " : failed to create GStreamer pipeline");
    return;
  }

  try {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);

    std::unique_lock<std::mutex> init_lock(g_rs_init_mutex);
    log("CAM RGB " + serial + " : starting (camera init serialized)");
    rs2::pipeline_profile profile = pipe.start(cfg);
    init_lock.unlock();

    // Publish the color intrinsics with zero latency: transient_local + reliable
    // latches K so a late-joining consumer sees it immediately, refreshed at ~1 Hz.
    // Same pattern as /oakd/camera_info (see depthai_720p_raw_streamer.cpp).
    auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = color_profile.get_intrinsics();
    auto ci = std::make_shared<sensor_msgs::msg::CameraInfo>();
    ci->header.frame_id = "d435_camera";
    ci->header.stamp = node->now();
    ci->width = intr.width;
    ci->height = intr.height;
    ci->distortion_model = "plumb_bob";  // D435 OEM cal is Brown-Conrady, 5 coeffs
    ci->k[0] = intr.fx; ci->k[2] = intr.ppx;
    ci->k[4] = intr.fy; ci->k[5] = intr.ppy;
    ci->k[8] = 1.0;
    ci->d.assign(intr.coeffs, intr.coeffs + 5);
    ci->r[0] = ci->r[4] = ci->r[8] = 1.0;  // identity rectification
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) ci->p[r * 4 + c] = ci->k[r * 3 + c];
      ci->p[r * 4 + 3] = 0.0;              // no rectified offset
    }
    cam_info_pub->publish(*ci);
    log("CAM RGB " + serial + " : published /d435/camera_info " +
        std::to_string(intr.width) + "x" + std::to_string(intr.height) +
        " fx=" + std::to_string(intr.fx) + " fy=" + std::to_string(intr.fy));

    // Same 25h9 board detector as the OAK-D tap, on its own thread, fed the
    // newest frame at ~5 Hz -> /d435/marker_board_pose{,_camera_frame},
    // /d435/apriltag_telemetry.
    std::unique_ptr<ffw_stream::BoardPoseDetector> tags;
    if (apriltag) {
      tags = std::make_unique<ffw_stream::BoardPoseDetector>(node, "/d435", "d435_camera", "d435");
      std::array<double, 9> K;
      std::copy(ci->k.begin(), ci->k.end(), K.begin());
      tags->set_intrinsics(K, std::vector<double>(ci->d.begin(), ci->d.end()));
      log("CAM RGB " + serial + " : AprilTag 25h9 board detector on (~5 Hz, own thread)");
    }
    int fps_frames = 0;
    auto fps_t0 = std::chrono::steady_clock::now();
    bool ts_domain_logged = false;

    size_t expected_size = width * height * 3;
    std::vector<uint8_t> rgb_buf(expected_size, 0);
    std::string cam_name = "RGB";
    auto last_info_pub = std::chrono::steady_clock::now();

    while (g_running && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        frames = pipe.wait_for_frames(5000);
      } catch (const rs2::error &e) {
        log("CAM RGB wait_for_frames error: " + std::string(e.what()));
        continue;
      }

      rs2::video_frame color = frames.get_color_frame();
      if (!color) continue;

      // Log latest hardware timestamp
      {
        std::lock_guard<std::mutex> lck(timestamp_mutex);
        latest_timestamps[cam_name] = frames.get_timestamp();
      }

      frames_captured[cam_name]++;

      const uint8_t *raw = reinterpret_cast<const uint8_t *>(color.get_data());
      int stride = color.get_stride_in_bytes();

      if (stride == width * 3) {
        std::memcpy(rgb_buf.data(), raw, expected_size);
      } else {
        for (int y = 0; y < height; ++y) {
          std::memcpy(rgb_buf.data() + y * width * 3, raw + y * stride, width * 3);
        }
      }

      // Push the frame into the GStreamer pipeline
      if (!gst_encoder_push_frame(gst_enc, rgb_buf.data(), expected_size)) {
        log("CAM RGB " + serial + " : gst_encoder_push_frame failed, stopping");
        break;
      }

      if (tags) {
        // Capture time on the system clock: RealSense GLOBAL/SYSTEM-domain
        // timestamps are host epoch ms; any other domain falls back to arrival.
        auto dom = color.get_frame_timestamp_domain();
        double cap_s = (dom == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME ||
                        dom == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)
            ? color.get_timestamp() / 1000.0
            : std::chrono::duration<double>(
                  std::chrono::system_clock::now().time_since_epoch()).count();
        if (!ts_domain_logged) {
          ts_domain_logged = true;
          log("CAM RGB " + serial + " : capture timestamps from domain '" +
              rs2_timestamp_domain_to_string(dom) + "'");
        }
        tags->submit_rgb(rgb_buf.data(), width, height, width * 3, cap_s);  // no-op unless due
        ++fps_frames;
        auto t = std::chrono::steady_clock::now();
        double el = std::chrono::duration<double>(t - fps_t0).count();
        if (el >= 1.0) {
          tags->set_camera_fps(fps_frames / el);
          fps_frames = 0;
          fps_t0 = t;
        }
      }

      // Refresh the latched camera_info at ~1 Hz so consumers see a live stamp
      // (the first publish already happened after pipe.start above).
      auto now_ci = std::chrono::steady_clock::now();
      if (now_ci - last_info_pub > std::chrono::milliseconds(1000)) {
        ci->header.stamp = node->now();
        cam_info_pub->publish(*ci);
        last_info_pub = now_ci;
      }
    }

    pipe.stop();
  } catch (const rs2::error &e) {
    log("CAM RGB FAILED (rs2::error): " + std::string(e.what()));
  } catch (const std::exception &e) {
    log("CAM RGB EXCEPTION: " + std::string(e.what()));
  }

  destroy_gst_stream(gst_enc);
  log("=== CAM RGB " + serial + " : stopped ===");
}

void stream_camera(const std::string &serial, int index,
                   const std::string &dest_ip, int depth_port, int ir_port,
                   int width, int height, int fps, float max_depth_m,
                   bool mjpeg = false, bool rgb_mode = false, int color_exposure_us = -1,
                   int color_wb = -1, bool enable_depth = true) {
  std::ostringstream hdr;
  hdr << "\n=== CAM" << index << " " << serial << " : "
      << (enable_depth ? ("depth->udp:" + std::to_string(depth_port) + "  ") : "depth=off  ")
      << (rgb_mode ? "rgb" : "ir") << "->udp:" << ir_port
      << "  gst=" << (mjpeg ? "mjpeg" : "h264") << " ===";
  log(hdr.str());

  GstEncoder depth_enc;
  if (enable_depth) {
    depth_enc = create_gst_stream(dest_ip, depth_port, width, height, fps, false, mjpeg);
  }
  GstEncoder second_enc = create_gst_stream(dest_ip, ir_port, width, height, fps, rgb_mode, mjpeg);

  if ((enable_depth && !depth_enc.pipeline) || !second_enc.pipeline) {
    log("CAM" + std::to_string(index) +
        " : failed to create GStreamer pipeline(s)");
    if (depth_enc.pipeline)
      destroy_gst_stream(depth_enc);
    if (second_enc.pipeline)
      destroy_gst_stream(second_enc);
    return;
  }

  try {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(serial);
    // enable_depth also gates whether the device is even asked for a depth
    // stream: `rs-enumerate-devices -c` confirms Color RGB8 at this
    // resolution is a valid standalone profile up to 90 Hz (does not require
    // depth to also be streaming), and 2x RGB (~22 MB/s) is less raw data
    // than the default IR+RGB+2xdepth profile (~30 MB/s) that already runs
    // at 30 fps — so there is no USB-bandwidth or profile-validity reason to
    // keep pulling depth over USB just because it won't be sent over the
    // network. A prior version of this code always enabled depth "to keep
    // fps up"; that diagnosis was wrong and just added unwanted USB traffic.
    if (enable_depth) {
      cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    }
    if (rgb_mode) {
      cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);
    } else {
      cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    }

    // Held through start + exposure/WB writes + restart (released below).
    std::unique_lock<std::mutex> init_lock(g_rs_init_mutex);
    log("CAM" + std::to_string(index) + " : starting (camera init serialized)");
    rs2::pipeline_profile profile = pipe.start(cfg);

    float depth_scale =
        profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    log("CAM" + std::to_string(index) +
        " depth_scale=" + std::to_string(depth_scale) + " m/unit");

    // Fixed exposure for the right D405 color stream: disable auto-exposure and
    // pin a low exposure (us) for a sharper, less-overbright image. Also disable
    // auto white balance — with a large dark area (gripper) in view, AWB gain
    // compensation can wash the image out.
    //
    // The D405 has no dedicated RGB module: its "color" stream is synthesized
    // from one of the global-shutter IR imagers, which is shared with the depth
    // stream. So `first<rs2::color_sensor>()` is not necessarily the sensor that
    // drives the streamed frames, and the depth sensor's auto-exposure can keep
    // overriding a manual exposure set only on the color sensor. We therefore:
    //   1. find the sensor that owns the active COLOR stream and pin its
    //      exposure / AWB,
    //   2. disable auto-exposure on the depth sensor too (same imager),
    //   3. restart the pipeline so the options are applied at stream start
    //      (some RealSense options only take effect when streaming begins).
    if (rgb_mode && color_exposure_us > 0) {
      try {
        rs2::device dev = profile.get_device();

        auto set_options = [&](rs2::sensor &s, const char *who) {
          if (s.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
            s.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
          if (s.supports(RS2_OPTION_EXPOSURE))
            s.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(color_exposure_us));
          if (s.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
            s.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0.0f);
          // Manual white balance: with AWB disabled the sensor sits at its
          // fixed WB, which reads yellow under warm light. Optionally pin a
          // manual value (K). Always log the supported range so we know the
          // valid scale even when not overriding.
          if (s.supports(RS2_OPTION_WHITE_BALANCE)) {
            rs2::option_range rng = s.get_option_range(RS2_OPTION_WHITE_BALANCE);
            if (color_wb > 0) {
              s.set_option(RS2_OPTION_WHITE_BALANCE, static_cast<float>(color_wb));
            }
            log("CAM" + std::to_string(index) + " " + who +
                " WB range: min=" + std::to_string(rng.min) +
                " max=" + std::to_string(rng.max) +
                " default=" + std::to_string(rng.def) +
                (color_wb > 0 ? "  manual_wb=" + std::to_string(color_wb)
                              : "  (left at default)"));
          } else {
            log("CAM" + std::to_string(index) + " " + who +
                " : RS2_OPTION_WHITE_BALANCE not supported");
          }
          float ae = s.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)
                         ? s.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE) : -1.0f;
          float exp = s.supports(RS2_OPTION_EXPOSURE)
                          ? s.get_option(RS2_OPTION_EXPOSURE) : -1.0f;
          float awb = s.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)
                          ? s.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) : -1.0f;
          float wb = s.supports(RS2_OPTION_WHITE_BALANCE)
                         ? s.get_option(RS2_OPTION_WHITE_BALANCE) : -1.0f;
          log("CAM" + std::to_string(index) + " " + who + " '" +
              std::string(s.get_info(RS2_CAMERA_INFO_NAME)) +
              "': auto_exposure=" + std::to_string(ae) +
              " exposure_us=" + std::to_string(exp) +
              " auto_white_balance=" + std::to_string(awb) +
              " white_balance=" + std::to_string(wb));
        };

        rs2::sensor color_owner;
        bool found = false;
        for (auto &s : dev.query_sensors()) {
          for (auto &sp : s.get_stream_profiles()) {
            if (sp.stream_type() == RS2_STREAM_COLOR) {
              color_owner = s;
              found = true;
              break;
            }
          }
          if (found) break;
        }
        if (found) {
          set_options(color_owner, "color");
        } else {
          log("CAM" + std::to_string(index) + " color: no sensor owns a COLOR stream");
        }

        // Same imager feeds depth; pin its auto-exposure off too so depth AE
        // cannot keep overriding the manual exposure above.
        try {
          rs2::depth_sensor ds = dev.first<rs2::depth_sensor>();
          set_options(ds, "depth");
        } catch (const rs2::error &e) {
          log("CAM" + std::to_string(index) + " depth exposure set failed: " + e.what());
        }

        // Restart so the options take effect at stream start. If this fails,
        // keep streaming with whatever stuck — just log it.
        try {
          pipe.stop();
          profile = pipe.start(cfg);
          log("CAM" + std::to_string(index) + " pipeline restarted after exposure set");
        } catch (const rs2::error &e) {
          log("CAM" + std::to_string(index) + " pipeline restart failed: " + e.what());
        }
      } catch (const rs2::error &e) {
        log("CAM" + std::to_string(index) + " color option set failed: " + e.what());
      }
    }
    init_lock.unlock();

    size_t second_bpp = rgb_mode ? 3 : 1;
    std::vector<uint8_t> depth8(width * height, 0);
    std::vector<uint8_t> second_buf(width * height * second_bpp, 0);
    std::string cam_name = "CAM" + std::to_string(index);

    while (g_running && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        frames = pipe.wait_for_frames(5000);
      } catch (const rs2::error &e) {
        log(cam_name + " wait_for_frames error: " + e.what());
        continue;
      }

      rs2::depth_frame depth = frames.get_depth_frame();
      if (enable_depth && !depth) continue;

      rs2::frame second_base;
      if (rgb_mode) {
        second_base = frames.get_color_frame();
      } else {
        second_base = frames.get_infrared_frame(1);
      }
      if (!second_base) continue;
      rs2::video_frame second_frame = second_base.as<rs2::video_frame>();
      if (!second_frame) continue;

      // Log latest hardware timestamp
      {
        std::lock_guard<std::mutex> lck(timestamp_mutex);
        latest_timestamps[cam_name] = frames.get_timestamp();
      }
      frames_captured[cam_name]++;

      // Depth → 8-bit
      if (enable_depth) {
        const uint16_t *draw = reinterpret_cast<const uint16_t *>(depth.get_data());
        int dstride = depth.get_stride_in_bytes() / 2;
        for (int y = 0; y < height; ++y) {
          const uint16_t *row = draw + y * dstride;
          uint8_t *out_row = depth8.data() + y * width;
          for (int x = 0; x < width; ++x) {
            float meters = row[x] * depth_scale;
            if (meters > max_depth_m) meters = max_depth_m;
            if (meters < 0.0f) meters = 0.0f;
            out_row[x] = static_cast<uint8_t>((meters / max_depth_m) * 255.0f + 0.5f);
          }
        }
      }

      // Second stream (IR or RGB) copy with stride handling
      const uint8_t *iraw = reinterpret_cast<const uint8_t *>(second_frame.get_data());
      int istride = second_frame.get_stride_in_bytes();
      if (rgb_mode) {
        if (istride == width * 3) {
          std::memcpy(second_buf.data(), iraw, second_buf.size());
        } else {
          for (int y = 0; y < height; ++y) {
            std::memcpy(second_buf.data() + y * width * 3, iraw + y * istride, width * 3);
          }
        }
      } else {
        for (int y = 0; y < height; ++y) {
          std::memcpy(second_buf.data() + y * width, iraw + y * istride, width);
        }
      }

      // Push frames into GStreamer pipelines
      if (enable_depth && !gst_encoder_push_frame(depth_enc, depth8.data(), depth8.size())) {
        log(cam_name + " : gst_encoder_push_frame(depth) failed, stopping");
        break;
      }
      if (!gst_encoder_push_frame(second_enc, second_buf.data(), second_buf.size())) {
        log(cam_name + " : gst_encoder_push_frame(second) failed, stopping");
        break;
      }
    }

    pipe.stop();
  } catch (const rs2::error &e) {
    log("CAM" + std::to_string(index) + " FAILED (rs2::error): " + e.what());
  } catch (const std::exception &e) {
    log("CAM" + std::to_string(index) + " EXCEPTION: " + e.what());
  }

  destroy_gst_stream(depth_enc);
  destroy_gst_stream(second_enc);
  log("=== CAM" + std::to_string(index) + " " + serial + " : stopped ===");
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <dest_ip> <base_port> [width=480] [height=270] [fps=30] "
                 "[max_depth_m=1.0] [--enable-d405s|--disable-d405s] "
                 "[--d435-rgb|--no-d435-rgb] [--dual-rgb-no-depth] "
                 "[--disable-left-d405] "
                 "[--color-exposure <us>] "
                 "[--color-wb <K>] [--d435-fps <n>] [--d435-codec mjpeg|h264] [--d435-bitrate <kbps>] "
                 "[--no-d435-apriltag] "
                 "[--h264|--mjpeg] (default H264)"
              << std::endl;
    return 1;
  }

  std::vector<std::string> positional_args;
  bool enable_d405s = true;
  bool d435_rgb_enabled = true;
  bool d435_apriltag = true;   // --no-d435-apriltag turns the board tap off
  bool dual_rgb_no_depth = false;  // profile: both D405s stream RGB, depth off
  bool disable_left_d405 = false;  // skip the left D405 (cam_idx 0) entirely
  int d435_fps = 0;            // 0 = follow the shared fps
  int d435_bitrate_kbps = 10000;  // h264 only: 720p@15 ~83 KB/frame, as the OAK-D's 20 Mbps @ 30
  // D435 head stream codec, independent of the D405s' --h264/--mjpeg: MJPEG
  // (jpegenc q90, same as the OAK-D MJPEG feed) by default -- every frame is
  // intra, so a lost packet costs that one frame instead of smearing blocks
  // until the next H264 keyframe.
  bool d435_mjpeg = true;
  bool mjpeg = false;          // default H264 (x264enc zerolatency); --mjpeg → jpegenc intra-only zero-latency
  int color_exposure_us = -1;  // -1 = leave SDK default auto-exposure
  int color_wb = -1;           // -1 = leave SDK default white balance
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--enable-d405s") {
      enable_d405s = true;
    } else if (arg == "--disable-d405s") {
      enable_d405s = false;
    } else if (arg == "--d435-rgb") {
      d435_rgb_enabled = true;
    } else if (arg == "--no-d435-rgb") {
      d435_rgb_enabled = false;
    } else if (arg == "--dual-rgb-no-depth") {
      dual_rgb_no_depth = true;
    } else if (arg == "--disable-left-d405") {
      disable_left_d405 = true;
    } else if (arg == "--color-exposure") {
      if (i + 1 < argc) {
        color_exposure_us = std::atoi(argv[++i]);
      }
    } else if (arg == "--color-wb") {
      if (i + 1 < argc) {
        color_wb = std::atoi(argv[++i]);
      }
    } else if (arg == "--no-d435-apriltag") {
      d435_apriltag = false;
    } else if (arg == "--d435-codec") {
      if (i + 1 < argc) {
        d435_mjpeg = std::string(argv[++i]) != "h264";
      }
    } else if (arg == "--d435-bitrate") {
      if (i + 1 < argc) {
        d435_bitrate_kbps = std::atoi(argv[++i]);
      }
    } else if (arg == "--d435-fps") {
      if (i + 1 < argc) {
        d435_fps = std::atoi(argv[++i]);
      }
    } else if (arg == "--mjpeg") {
      mjpeg = true;
    } else if (arg == "--h264") {
      mjpeg = false;
    } else {
      positional_args.push_back(arg);
    }
  }

  if (positional_args.size() < 2) {
    std::cerr << "Missing required positional arguments." << std::endl;
    return 1;
  }

  std::string dest_ip = positional_args[0];
  int base_port = std::atoi(positional_args[1].c_str());

  int width = 480;
  int height = 270;
  int fps = 15;  // revamp: everything 15Hz, no more 30Hz anywhere
  float max_depth_m = 1.0f;

  if (positional_args.size() > 2) width = std::atoi(positional_args[2].c_str());
  if (positional_args.size() > 3) height = std::atoi(positional_args[3].c_str());
  if (positional_args.size() > 4) fps = std::atoi(positional_args[4].c_str());
  if (positional_args.size() > 5) max_depth_m = static_cast<float>(std::atof(positional_args[5].c_str()));

  if (width == 0) width = 480;
  if (height == 0) height = 270;
  if (fps == 0) fps = 15;
  if (d435_fps <= 0) d435_fps = fps;
  if (max_depth_m <= 0) max_depth_m = 1.0f;

  std::signal(SIGINT, on_sigint);

  gst_init(nullptr, nullptr);

  // Turn this executable into a ROS node so the D435 RGB stream can publish its
  // intrinsics on /d435/camera_info with zero latency (transient_local, reliable).
  // rclcpp's SIGINT handler makes rclcpp::ok() go false on Ctrl-C; the streaming
  // threads check it alongside g_running so shutdown completes.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("realsense_udp_streamer");
  auto cam_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/d435/camera_info", rclcpp::QoS(1).transient_local().reliable());
  // Spin for the board detector's /head_camera_tf subscription.
  std::thread([node] { rclcpp::spin(node); }).detach();

  rs2::context ctx;
  auto devices = ctx.query_devices();
  if (devices.size() == 0) {
    std::cerr << "No devices found!" << std::endl;
    return 1;
  }

  // The head D435/D435i (USB PID 0B07 / 0B3A) is told apart from the D405
  // hand cameras (0B5B) by product ID, falling back to the model name.
  auto is_d435 = [](const rs2::device &dev) {
    std::string pid = dev.supports(RS2_CAMERA_INFO_PRODUCT_ID)
                          ? dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) : "";
    std::transform(pid.begin(), pid.end(), pid.begin(), ::toupper);
    if (pid == "0B07" || pid == "0B3A") return true;
    std::string name = dev.supports(RS2_CAMERA_INFO_NAME)
                           ? dev.get_info(RS2_CAMERA_INFO_NAME) : "";
    return name.find("D435") != std::string::npos;
  };

  std::vector<std::string> serials;
  std::vector<bool> d435_flags;
  for (size_t i = 0; i < devices.size(); ++i) {
    std::string serial =
        devices[i].supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
            ? devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
            : "";
    serials.push_back(serial);
    d435_flags.push_back(is_d435(devices[i]));
  }
  
  // Reverse the default SDK enumeration order so Left becomes 0 and Right becomes 1
  std::reverse(serials.begin(), serials.end());
  std::reverse(d435_flags.begin(), d435_flags.end());

  log("Destination: " + dest_ip + "  base_port=" + std::to_string(base_port) +
      "  " + std::to_string(width) + "x" + std::to_string(height) + "@" +
      std::to_string(fps) + "  max_depth=" + std::to_string(max_depth_m) + "m" +
      "  codec=" + std::string(mjpeg ? "mjpeg" : "h264") +
      "  d405s=" + std::string(enable_d405s ? "on" : "off") +
      "  d435_rgb=" + std::string(d435_rgb_enabled ? "on" : "off") +
      "  d435_fps=" + std::to_string(d435_fps) +
      "  d435_codec=" + std::string(d435_mjpeg ? "mjpeg" : "h264") +
      "  dual_rgb_no_depth=" + std::string(dual_rgb_no_depth ? "on" : "off") +
      "  disable_left_d405=" + std::string(disable_left_d405 ? "on" : "off") +
      "  color_exposure_us=" + std::to_string(color_exposure_us) +
      "  color_wb=" + std::to_string(color_wb));

  std::vector<std::thread> threads;
  int cam_idx = 0;
  for (size_t i = 0; i < serials.size(); ++i) {
    if (d435_flags[i]) {
      if (d435_rgb_enabled) {
        // The D435i replacing the ZED: Stream 720p RGB to the ZED's port
        int rgb_port = base_port + 100;
        frames_captured["RGB"];
        threads.emplace_back(stream_camera_rgb, serials[i], dest_ip, rgb_port,
                             1280, 720, d435_fps, d435_mjpeg, node, cam_info_pub, d435_apriltag,
                             d435_bitrate_kbps);
      } else {
        log("CAM RGB " + serials[i] + " (D435): --no-d435-rgb -- not opened");
      }
    } else if (enable_d405s) {
      if (disable_left_d405 && cam_idx == 0) {
        // Left D405: not opened at all -- zero USB/compute cost from it.
        log("CAM0 (left D405): disabled via --disable-left-d405 -- not opened");
        cam_idx++;
        continue;
      }
      int depth_port = base_port + cam_idx * 2;
      int ir_port = depth_port + 1;
      // Both D405s always stream RGB now (no more left=IR profile) -- at
      // 15Hz, both-RGB+both-depth fits comfortably under the USB budget the
      // old 30fps legacy default already used. dual_rgb_no_depth now purely
      // toggles depth capture/transmission on/off; it no longer changes
      // which cameras are RGB, since that's unconditional.
      bool rgb_mode = true;
      bool enable_depth = !dual_rgb_no_depth;
      frames_captured["CAM" + std::to_string(cam_idx)];
      threads.emplace_back(stream_camera, serials[i], cam_idx,
                           dest_ip, depth_port, ir_port, width, height, fps,
                           max_depth_m, mjpeg, rgb_mode, color_exposure_us, color_wb,
                           enable_depth);
      cam_idx++;
    }
  }

  for (auto &t : threads) {
    t.detach();
  }

  // Monitor Loop Setup
  auto last_print_time = std::chrono::steady_clock::now();
  std::map<std::string, int> last_frame_counts;

  // Telemetry UDP Socket
  int telemetry_sock = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in telemetry_addr;
  memset(&telemetry_addr, 0, sizeof(telemetry_addr));
  telemetry_addr.sin_family = AF_INET;
  telemetry_addr.sin_port = htons(base_port + 200);
  inet_pton(AF_INET, dest_ip.c_str(), &telemetry_addr.sin_addr);

  // Startup watchdog: if NO camera has produced a single frame this long after
  // start, the bring-up is hung (see g_rs_init_mutex) -- say so and exit so
  // the launch reports the process died instead of streaming nothing silently.
  constexpr double kNoFrameExitS = 30.0;
  const auto start_time = std::chrono::steady_clock::now();
  bool any_frame = false;

  while (g_running && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(5));

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_print_time).count();

    if (!any_frame && !frames_captured.empty()) {
      for (const auto& [cam, count] : frames_captured) {
        if (count.load() > 0) any_frame = true;
      }
      double since_start = std::chrono::duration<double>(now - start_time).count();
      if (!any_frame && since_start > kNoFrameExitS) {
        log("FATAL: no frames from any camera " + std::to_string(int(since_start)) +
            " s after start -- RealSense init hung; exiting (restart the stream)");
        std::_Exit(2);  // camera threads may be deadlocked inside librealsense
      }
    }
    
    std::ostringstream ss;
    ss << "FPS -> ";
    for (const auto& [cam, count] : frames_captured) {
      int current = count.load();
      int delta = current - last_frame_counts[cam];
      last_frame_counts[cam] = current;
      double fps_val = delta / elapsed;
      ss << cam << ": " << std::fixed << std::setprecision(1) << fps_val << " | ";
    }

    {
      std::lock_guard<std::mutex> lck(timestamp_mutex);
      if (latest_timestamps.size() > 0) {
        double min_t = -1.0;
        double max_t = -1.0;
        for (const auto& kv : latest_timestamps) {
          if (min_t < 0 || kv.second < min_t) min_t = kv.second;
          if (max_t < 0 || kv.second > max_t) max_t = kv.second;
        }
        double worst_delay = max_t - min_t;
        ss << "Worst Delay: " << std::fixed << std::setprecision(1) << worst_delay << " ms";
      }
    }
    
    std::string telemetry_msg = ss.str();
    if (telemetry_sock >= 0) {
      sendto(telemetry_sock, telemetry_msg.c_str(), telemetry_msg.length(), 0,
             (struct sockaddr*)&telemetry_addr, sizeof(telemetry_addr));
    }
    
    last_print_time = now;
  }

  if (telemetry_sock >= 0) {
    close(telemetry_sock);
  }

  rclcpp::shutdown();
  log("\n=== Done ===");
  return 0;
}