// udp_depth_ir_streamer.cpp
//
// For each connected RealSense camera:
//   - Opens Depth + IR(1) streams.
//   - Clamps depth to [0, max_depth_m] meters and rescales to 8-bit (0-255).
//   - IR frame is already 8-bit (Y8), copied out respecting row stride.
//   - Streams both 8-bit images as separate UDP streams via ffmpeg
//     (MJPEG-in-MPEGTS), one ffmpeg subprocess + one UDP port per image.
//
// NOTE on depth=0: the RealSense SDK uses raw value 0 to mean "no valid
// depth" (sensor couldn't measure that pixel). With this mapping that also
// comes out as 0 in the 8-bit image -- visually identical to "actual
// distance = 0m". If you need to tell those apart downstream, special-case
// raw==0 before scaling (e.g. force it to 255) and document the convention
// for whoever consumes the stream.
//
// Usage:
//   udp_depth_ir_streamer <dest_ip> <base_port> [width=480] [height=270]
//   [fps=30] [max_depth_m=1.0] [--enable-d405s|--disable-d405s]
//   [--d435-rgb|--no-d435-rgb]
//
//   codec=mjpeg (default) - libjpeg-based, intra-only, each frame independent
//                           (a lost packet only corrupts one frame). Higher
//                           bandwidth.
//   codec=h264             - libx264 software encode (preset ultrafast, tune
//   zerolatency).
//                           Much lower bandwidth, but frames depend on previous
//                           frames, so packet loss can smear until the next
//                           keyframe (-g controls keyframe interval, set to fps
//                           below so a fresh keyframe arrives every ~1s).
//
// Port mapping (per camera index i, 0-based):
//   depth -> base_port + i*2
//   ir    -> base_port + i*2 + 1
//
// Requires ffmpeg to be installed and on PATH.
// Build: g++ -std=c++17 udp_depth_ir_streamer.cpp -lrealsense2 -lpthread -o
// udp_depth_ir_streamer

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdio>
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

// Spawn an ffmpeg process that reads raw 8-bit grayscale frames from stdin
// and streams them over UDP to ip:port, using either MJPEG or libx264.
FILE *open_ffmpeg_sender(const std::string &ip, int port, int width, int height,
                         int fps) {
  std::ostringstream cmd;
  cmd << "ffmpeg -hide_banner -loglevel error "
      << "-f rawvideo -pixel_format gray -video_size " << width << "x" << height
      << " -framerate " << fps << " -i - ";

  cmd << "-c:v libx264 -preset ultrafast -tune zerolatency "
      << "-x264opts slice-max-size=1200 "
      << "-g 10 -pix_fmt yuv420p ";

  cmd << "-f rtp rtp://" << ip << ":" << port << "?pkt_size=1316";
  log("  ffmpeg cmd: " + cmd.str());
  return popen(cmd.str().c_str(), "w");
}

FILE *open_ffmpeg_sender_rgb(const std::string &ip, int port, int width, int height,
                             int fps) {
  std::ostringstream cmd;
  cmd << "ffmpeg -hide_banner -loglevel error "
      << "-f rawvideo -pixel_format rgb24 -video_size " << width << "x" << height
      << " -framerate " << fps << " -i - ";

  cmd << "-c:v libx264 -preset ultrafast -tune zerolatency "
      << "-x264opts slice-max-size=1200 "
      << "-g " << fps << " -pix_fmt yuv420p ";

  cmd << "-f rtp rtp://" << ip << ":" << port << "?pkt_size=1316";
  log("  ffmpeg rgb cmd: " + cmd.str());
  return popen(cmd.str().c_str(), "w");
}

void stream_camera_rgb(const std::string &serial, const std::string &dest_ip, int port,
                       int width, int height, int fps) {
  std::ostringstream hdr;
  hdr << "\n=== CAM (RGB ZED-Replacement) " << serial << " : rgb->udp:" << port
      << "  codec=h264 ===";
  log(hdr.str());

  FILE *rgb_pipe = open_ffmpeg_sender_rgb(dest_ip, port, width, height, fps);

  if (!rgb_pipe) {
    log("CAM RGB " + serial + " : failed to start ffmpeg sender process");
    return;
  }

  try {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);

    pipe.start(cfg);

    size_t expected_size = width * height * 3;
    std::vector<uint8_t> rgb_buf(expected_size, 0);
    std::string cam_name = "RGB";

    while (g_running) {
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
      
      // Send the frame
      size_t written = fwrite(rgb_buf.data(), 1, expected_size, rgb_pipe);
      fflush(rgb_pipe);
      if (written != expected_size) break;
    }

    pipe.stop();
  } catch (const rs2::error &e) {
    log("CAM RGB FAILED (rs2::error): " + std::string(e.what()));
  } catch (const std::exception &e) {
    log("CAM RGB EXCEPTION: " + std::string(e.what()));
  }

  pclose(rgb_pipe);
  log("=== CAM RGB " + serial + " : stopped ===");
}

void stream_camera(const std::string &serial, int index,
                   const std::string &dest_ip, int depth_port, int ir_port,
                   int width, int height, int fps, float max_depth_m) {
  std::ostringstream hdr;
  hdr << "\n=== CAM" << index << " " << serial << " : depth->udp:" << depth_port
      << "  ir->udp:" << ir_port << "  codec=h264 ===";
  log(hdr.str());

  FILE *depth_pipe =
      open_ffmpeg_sender(dest_ip, depth_port, width, height, fps);
  FILE *ir_pipe =
      open_ffmpeg_sender(dest_ip, ir_port, width, height, fps);

  if (!depth_pipe || !ir_pipe) {
    log("CAM" + std::to_string(index) +
        " : failed to start ffmpeg sender process(es)");
    if (depth_pipe)
      pclose(depth_pipe);
    if (ir_pipe)
      pclose(ir_pipe);
    return;
  }

  try {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8,
                      fps);

    rs2::pipeline_profile profile = pipe.start(cfg);

    // meters per depth unit -- query rather than assume (varies by
    // sensor/preset)
    float depth_scale =
        profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    log("CAM" + std::to_string(index) +
        " depth_scale=" + std::to_string(depth_scale) + " m/unit");

    std::vector<uint8_t> depth8(width * height, 0);
    std::vector<uint8_t> ir8(width * height, 0);
    std::string cam_name = "CAM" + std::to_string(index);

    while (g_running) {
      rs2::frameset frames;
      try {
        frames = pipe.wait_for_frames(5000);
      } catch (const rs2::error &e) {
        log(cam_name + " wait_for_frames error: " + e.what());
        continue;
      }

      rs2::depth_frame depth = frames.get_depth_frame();
      rs2::video_frame ir = frames.get_infrared_frame(1);

      if (!depth || !ir) continue;

      // Log latest hardware timestamp
      {
        std::lock_guard<std::mutex> lck(timestamp_mutex);
        latest_timestamps[cam_name] = frames.get_timestamp();
      }

      frames_captured[cam_name]++;

      const uint16_t *draw = reinterpret_cast<const uint16_t *>(depth.get_data());
      int dstride = depth.get_stride_in_bytes() / 2; // uint16 units per row
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

      const uint8_t *iraw = reinterpret_cast<const uint8_t *>(ir.get_data());
      int istride = ir.get_stride_in_bytes();
      for (int y = 0; y < height; ++y) {
        std::memcpy(ir8.data() + y * width, iraw + y * istride, width);
      }

      size_t dwritten = fwrite(depth8.data(), 1, depth8.size(), depth_pipe);
      size_t iwritten = fwrite(ir8.data(), 1, ir8.size(), ir_pipe);
      fflush(depth_pipe);
      fflush(ir_pipe);

      if (dwritten != depth8.size() || iwritten != ir8.size()) {
        log(cam_name + " : short write to ffmpeg pipe, stopping");
        break;
      }
    }

    pipe.stop();
  } catch (const rs2::error &e) {
    log("CAM" + std::to_string(index) + " FAILED (rs2::error): " + e.what());
  } catch (const std::exception &e) {
    log("CAM" + std::to_string(index) + " EXCEPTION: " + e.what());
  }

  pclose(depth_pipe);
  pclose(ir_pipe);
  log("=== CAM" + std::to_string(index) + " " + serial + " : stopped ===");
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <dest_ip> <base_port> [width=480] [height=270] [fps=30] "
                 "[max_depth_m=1.0] [--enable-d405s|--disable-d405s] "
                 "[--d435-rgb|--no-d435-rgb]"
              << std::endl;
    return 1;
  }

  std::vector<std::string> positional_args;
  bool enable_d405s = true;
  bool d435_rgb_enabled = true;
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
      "  d405s=" + std::string(enable_d405s ? "on" : "off") +
      "  d435_rgb=" + std::string(d435_rgb_enabled ? "on" : "off"));

  std::vector<std::thread> threads;
  int cam_idx = 0;
  for (size_t i = 0; i < serials.size(); ++i) {
    if (serials[i] == "941322072865") {
      if (d435_rgb_enabled) {
        // The D435i replacing the ZED: Stream 1080p RGB to the ZED's port
        int rgb_port = base_port + 100;
        threads.emplace_back(stream_camera_rgb, serials[i], dest_ip, rgb_port, 1920, 1080, 30);
      }
    } else if (enable_d405s) {
      int depth_port = base_port + cam_idx * 2;
      int ir_port = depth_port + 1;
      threads.emplace_back(stream_camera, serials[i], cam_idx,
                           dest_ip, depth_port, ir_port, width, height, fps,
                           max_depth_m);
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

  while (g_running) {
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

  log("\n=== Done ===");
  return 0;
}