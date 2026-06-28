// zed_udp_streamer.cpp
//
// Streams ZED M Left camera via UDP using ffmpeg (H264).
// Usage: zed_udp_streamer <dest_ip> <base_port> [fps=30]
// The streaming port will be base_port + 100 as requested.

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>

#include "zed_udp_streamer.hpp"

std::mutex cout_mutex;
std::atomic<bool> g_running{true};

void log(const std::string &msg) {
  std::lock_guard<std::mutex> lock(cout_mutex);
  std::cout << msg << std::endl;
}

void on_sigint(int) { g_running = false; }

FILE *open_ffmpeg_sender(const std::string &ip, int port, int width, int height,
                         int fps) {
  std::ostringstream cmd;
  cmd << "ffmpeg -hide_banner -loglevel error "
      << "-f rawvideo -pixel_format bgra -video_size " << width << "x" << height
      << " -framerate " << fps << " -i - ";

  cmd << "-c:v libx264 -preset ultrafast -tune zerolatency "
      << "-x264opts slice-max-size=1200 "
      << "-g " << fps << " -pix_fmt yuv420p ";

  cmd << "-f rtp rtp://" << ip << ":" << port << "?pkt_size=1316";
  log("  ffmpeg cmd: " + cmd.str());
  return popen(cmd.str().c_str(), "w");
}

extern "C" int start_stream(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <dest_ip> <base_port> [fps=30] [res=720|1080]"
              << std::endl;
    return 1;
  }

  std::string dest_ip = argv[1];
  int base_port = std::atoi(argv[2]);
  int stream_port = base_port + 100; // As requested: base_port + 100

  int fps = 30;
  if (argc > 3)
    fps = std::atoi(argv[3]);
  if (fps <= 0)
    fps = 30;

  int res = 720;
  if (argc > 4)
    res = std::atoi(argv[4]);

  std::signal(SIGINT, on_sigint);

  sl::Camera zed;
  sl::InitParameters init_params;
  
  if (res == 1080) {
    init_params.camera_resolution = sl::RESOLUTION::HD1080;
  } else {
    init_params.camera_resolution = sl::RESOLUTION::HD720;
  }
  
  init_params.camera_fps = fps;
  // Minimize depth processing to save resources on the host since we only want
  // the left camera
  init_params.depth_mode = sl::DEPTH_MODE::NONE;

  sl::ERROR_CODE err = zed.open(init_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    std::cerr << "Failed to open ZED Camera: " << sl::toString(err)
              << std::endl;
    return 1;
  }

  int width = (res == 1080) ? 1920 : 1280;
  int height = (res == 1080) ? 1080 : 720;

  log("Opened ZED Camera: " + std::to_string(width) + "x" +
      std::to_string(height) + " @ " + std::to_string(fps) + " fps");
  log("Destination: " + dest_ip +
      "  stream_port=" + std::to_string(stream_port) + "  codec=h264");

  FILE *stream_pipe =
      open_ffmpeg_sender(dest_ip, stream_port, width, height, fps);
  if (!stream_pipe) {
    log("Failed to start ffmpeg sender process");
    zed.close();
    return 1;
  }

  sl::Mat image_left;
  int frame_count = 0;

  log("Starting video streaming loop...");
  while (g_running) {
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
      zed.retrieveImage(image_left, sl::VIEW::LEFT);
      
      // Wrap the sl::Mat memory in an OpenCV cv::Mat so we can draw text directly onto the buffer
      cv::Mat cv_frame(height, width, CV_8UC4, image_left.getPtr<sl::uchar1>(), image_left.getStepBytes());
      
      std::string text = "FRAME " + std::to_string(frame_count);
      // Draw at the very top
      cv::putText(cv_frame, text, cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 0, 255), 5);
      // Draw at the very bottom
      cv::putText(cv_frame, text, cv::Point(50, height - 100), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 0, 255, 255), 5);

      size_t expected_size = width * height * 4;
      size_t written = fwrite(image_left.getPtr<sl::uchar1>(), 1, expected_size, stream_pipe);
      if (written != expected_size) {
        log("Short write to ffmpeg pipe, stopping");
        break;
      }
      fflush(stream_pipe);
      frame_count++;

      if (frame_count % 150 == 0) {
        log("ZED streamed " + std::to_string(frame_count) + " frames");
      }
    } else {
      // Small sleep if grab fails to prevent CPU spinning, though grab()
      // usually blocks
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  log("Stopping streamer...");
  zed.close();
  pclose(stream_pipe);
  log("=== Done ===");
  return 0;
}
