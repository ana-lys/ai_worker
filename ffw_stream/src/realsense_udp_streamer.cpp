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
//   [fps=30] [max_depth_m=1.0] [codec=mjpeg|h264]
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

std::mutex cout_mutex;
std::atomic<bool> g_running{true};

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
      << "-g 10 -pix_fmt yuv420p ";

  cmd << "-f rtp rtp://" << ip << ":" << port << "?pkt_size=1316";
  log("  ffmpeg cmd: " + cmd.str());
  return popen(cmd.str().c_str(), "w");
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

    std::vector<uint8_t> depth8(width * height);
    std::vector<uint8_t> ir8(width * height);

    uint64_t frame_count = 0;

    while (g_running) {
      rs2::frameset frames;
      try {
        frames = pipe.wait_for_frames(5000);
      } catch (const rs2::error &e) {
        log("CAM" + std::to_string(index) +
            " wait_for_frames error: " + e.what());
        continue;
      }

      rs2::depth_frame depth = frames.get_depth_frame();
      rs2::video_frame ir = frames.get_infrared_frame(1);

      if (!depth || !ir)
        continue;

      // ---- Depth: clamp to [0, max_depth_m], scale to 8-bit ----
      const uint16_t *draw =
          reinterpret_cast<const uint16_t *>(depth.get_data());
      int dstride = depth.get_stride_in_bytes() / 2; // uint16 units per row
      for (int y = 0; y < height; ++y) {
        const uint16_t *row = draw + y * dstride;
        uint8_t *out_row = depth8.data() + y * width;
        for (int x = 0; x < width; ++x) {
          float meters = row[x] * depth_scale;
          if (meters > max_depth_m)
            meters = max_depth_m;
          if (meters < 0.0f)
            meters = 0.0f;
          out_row[x] =
              static_cast<uint8_t>((meters / max_depth_m) * 255.0f + 0.5f);
        }
      }

      // ---- IR: already 8-bit, just respect stride ----
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
        log("CAM" + std::to_string(index) +
            " : short write to ffmpeg pipe, stopping");
        break;
      }

      if (++frame_count % 150 == 0) {
        log("CAM" + std::to_string(index) + " streamed " +
            std::to_string(frame_count) + " frames");
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
                 "[max_depth_m=1.0]"
              << std::endl;
    return 1;
  }

  std::string dest_ip = argv[1];
  int base_port = std::atoi(argv[2]);

  int width = 480;
  int height = 270;
  int fps = 30;
  float max_depth_m = 1.0f;

  if (argc > 3 && std::string(argv[3]) != "--ros-args") width = std::atoi(argv[3]);
  if (argc > 4 && std::string(argv[4]) != "--ros-args") height = std::atoi(argv[4]);
  if (argc > 5 && std::string(argv[5]) != "--ros-args") fps = std::atoi(argv[5]);
  if (argc > 6 && std::string(argv[6]) != "--ros-args") max_depth_m = static_cast<float>(std::atof(argv[6]));

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
  
  // Sort serials to ensure deterministic ordering (e.g., Left vs Right)
  std::sort(serials.begin(), serials.end());

  log("Destination: " + dest_ip + "  base_port=" + std::to_string(base_port) +
      "  " + std::to_string(width) + "x" + std::to_string(height) + "@" +
      std::to_string(fps) + "  max_depth=" + std::to_string(max_depth_m) + "m");

  std::vector<std::thread> threads;
  for (size_t i = 0; i < serials.size(); ++i) {
    int depth_port = base_port + static_cast<int>(i) * 2;
    int ir_port = depth_port + 1;
    threads.emplace_back(stream_camera, serials[i], static_cast<int>(i),
                         dest_ip, depth_port, ir_port, width, height, fps,
                         max_depth_m);
  }

  for (auto &t : threads)
    t.join();

  log("\n=== Done ===");
  return 0;
}