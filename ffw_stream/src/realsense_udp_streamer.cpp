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
//   [--d435-rgb|--no-d435-rgb] [--color-exposure <microseconds>]
//
// Port mapping (per camera index i, 0-based):
//   depth -> base_port + i*2
//   ir    -> base_port + i*2 + 1
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

// GStreamer appsrc-based H264 encoding
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// ROS node so this executable can publish the D435 intrinsics with zero-latency
// (transient_local) QoS — same pattern as the OAK-D streamer's /oakd/camera_info.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

std::mutex cout_mutex;
std::atomic<bool> g_running{true};

// Monitor variables
std::map<std::string, std::atomic<int>> frames_captured;
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
GstEncoder create_gst_stream(const std::string &ip, int port, int width, int height,
                            int fps, bool rgb_mode, bool mjpeg) {
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
    pipe << "x264enc speed-preset=ultrafast tune=zerolatency key-int-max=" << fps << " ! "
         << "h264parse config-interval=-1 ! "
         << "rtph264pay pt=96 ! "
         << "udpsink host=" << ip << " port=" << port << " sync=false async=false";
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
                       rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub) {
  std::ostringstream hdr;
  hdr << "\n=== CAM (RGB ZED-Replacement) " << serial << " : rgb->udp:" << port
      << "  gst=" << (mjpeg ? "mjpeg" : "h264") << " ===";
  log(hdr.str());

  GstEncoder gst_enc = create_gst_stream(dest_ip, port, width, height, fps, true, mjpeg);

  if (!gst_enc.pipeline) {
    log("CAM RGB " + serial + " : failed to create GStreamer pipeline");
    return;
  }

  try {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);

    rs2::pipeline_profile profile = pipe.start(cfg);

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
                   int color_wb = -1) {
  std::ostringstream hdr;
  hdr << "\n=== CAM" << index << " " << serial << " : depth->udp:" << depth_port
      << "  " << (rgb_mode ? "rgb" : "ir") << "->udp:" << ir_port
      << "  gst=" << (mjpeg ? "mjpeg" : "h264") << " ===";
  log(hdr.str());

  GstEncoder depth_enc = create_gst_stream(dest_ip, depth_port, width, height, fps, false, mjpeg);
  GstEncoder second_enc = create_gst_stream(dest_ip, ir_port, width, height, fps, rgb_mode, mjpeg);

  if (!depth_enc.pipeline || !second_enc.pipeline) {
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
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    if (rgb_mode) {
      cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);
    } else {
      cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    }

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
      if (!depth) continue;

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
      if (!gst_encoder_push_frame(depth_enc, depth8.data(), depth8.size())) {
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
                 "[--d435-rgb|--no-d435-rgb] [--color-exposure <us>] "
                 "[--color-wb <K>] [--mjpeg|--h264] (default MJPEG)"
              << std::endl;
    return 1;
  }

  std::vector<std::string> positional_args;
  bool enable_d405s = true;
  bool d435_rgb_enabled = true;
  bool mjpeg = true;           // default MJPEG (jpegenc, intra-only zero-latency); --h264 → x264enc
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
    } else if (arg == "--color-exposure") {
      if (i + 1 < argc) {
        color_exposure_us = std::atoi(argv[++i]);
      }
    } else if (arg == "--color-wb") {
      if (i + 1 < argc) {
        color_wb = std::atoi(argv[++i]);
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
  int fps = 30;
  float max_depth_m = 1.0f;

  if (positional_args.size() > 2) width = std::atoi(positional_args[2].c_str());
  if (positional_args.size() > 3) height = std::atoi(positional_args[3].c_str());
  if (positional_args.size() > 4) fps = std::atoi(positional_args[4].c_str());
  if (positional_args.size() > 5) max_depth_m = static_cast<float>(std::atof(positional_args[5].c_str()));

  if (width == 0) width = 480;
  if (height == 0) height = 270;
  if (fps == 0) fps = 30;
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

  rs2::context ctx;
  auto devices = ctx.query_devices();
  if (devices.size() == 0) {
    std::cerr << "No devices found!" << std::endl;
    return 1;
  }

  std::vector<std::string> serials;
  for (size_t i = 0; i < devices.size(); ++i) {
    std::string serial =
        devices[i].supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
            ? devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
            : "";
    serials.push_back(serial);
  }
  
  // Reverse the default SDK enumeration order so Left becomes 0 and Right becomes 1
  std::reverse(serials.begin(), serials.end());

  log("Destination: " + dest_ip + "  base_port=" + std::to_string(base_port) +
      "  " + std::to_string(width) + "x" + std::to_string(height) + "@" +
      std::to_string(fps) + "  max_depth=" + std::to_string(max_depth_m) + "m" +
      "  codec=" + std::string(mjpeg ? "mjpeg" : "h264") +
      "  d405s=" + std::string(enable_d405s ? "on" : "off") +
      "  d435_rgb=" + std::string(d435_rgb_enabled ? "on" : "off") +
      "  color_exposure_us=" + std::to_string(color_exposure_us) +
      "  color_wb=" + std::to_string(color_wb));

  std::vector<std::thread> threads;
  int cam_idx = 0;
  for (size_t i = 0; i < serials.size(); ++i) {
    if (serials[i] == "941322072865") {
      if (d435_rgb_enabled) {
        // The D435i replacing the ZED: Stream 720p RGB to the ZED's port
        int rgb_port = base_port + 100;
        threads.emplace_back(stream_camera_rgb, serials[i], dest_ip, rgb_port,
                             1280, 720, 30, mjpeg, node, cam_info_pub);
      }
    } else if (enable_d405s) {
      int depth_port = base_port + cam_idx * 2;
      int ir_port = depth_port + 1;
      // Right D405 (cam_idx==1) streams RGB instead of IR at 480×270
      bool rgb_mode = (cam_idx == 1);
      threads.emplace_back(stream_camera, serials[i], cam_idx,
                           dest_ip, depth_port, ir_port, width, height, fps,
                           max_depth_m, mjpeg, rgb_mode, color_exposure_us, color_wb);
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

  while (g_running && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(5));

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_print_time).count();
    
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