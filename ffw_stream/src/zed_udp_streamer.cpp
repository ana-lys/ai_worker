// zed_udp_streamer.cpp
//
// Streams ZED M Left camera via UDP using ffmpeg (H264).
// Usage: zed_udp_streamer <dest_ip> <base_port> [fps=30]
// The streaming port will be base_port + 100 as requested.

#include <atomic>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

#include "zed_udp_streamer.hpp"

std::atomic<bool> g_running(true);

void signalHandler(int signum) {
  g_running = false;
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
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

  log("Starting single-thread video streaming loop with Algorithmic Fixer...");
  
  int frame_count = 0;
  sl::Mat image_left;
  
  while (g_running) {
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
      zed.retrieveImage(image_left, sl::VIEW::LEFT);
      
      cv::Mat cv_frame(height, width, CV_8UC4, image_left.getPtr<sl::uchar1>(), image_left.getStepBytes());
      
      // Draw diagnostic text LEFT and RIGHT
      std::string l_text = "LEFT " + std::to_string(frame_count);
      std::string r_text = "RIGHT " + std::to_string(frame_count);
      cv::putText(cv_frame, l_text, cv::Point(50, height / 2), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 0, 255), 5);
      cv::putText(cv_frame, r_text, cv::Point(width - 400, height / 2), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 0, 255, 255), 5);

      // Algorithmic Vertical Tear Detector
      // Downsample heavily for 1ms fast detection
      cv::Mat small;
      cv::resize(cv_frame, small, cv::Size(128, 64)); 
      cv::cvtColor(small, small, cv::COLOR_BGRA2GRAY);

      int best_x = -1;
      double max_diff = 0;
      
      // Only search the middle 50% of the image to avoid borders
      for(int x = small.cols/4; x < 3*small.cols/4; ++x) {
          cv::Mat diff;
          cv::absdiff(small.col(x), small.col(x-1), diff);
          double d = cv::mean(diff)[0];
          if (d > max_diff) {
              max_diff = d;
              best_x = x;
          }
      }

      uint8_t* pipe_data = image_left.getPtr<sl::uchar1>();
      cv::Mat fixed_frame;

      // Threshold for unnatural vertical discontinuity
      if (max_diff > 35.0) {
          int real_x = (best_x * width) / small.cols;
          
          log("Hardware Tear Detected at X=" + std::to_string(real_x) + " (Severity: " + std::to_string(max_diff) + "). Un-swapping memory!");

          // Fix the swap
          fixed_frame = cv::Mat(cv_frame.size(), cv_frame.type());
          cv_frame.colRange(real_x, width).copyTo(fixed_frame.colRange(0, width - real_x));
          cv_frame.colRange(0, real_x).copyTo(fixed_frame.colRange(width - real_x, width));
          
          pipe_data = fixed_frame.data;
      }

      size_t expected_size = width * height * 4;
      size_t written = fwrite(pipe_data, 1, expected_size, stream_pipe);
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
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  log("Stopping streamer...");
  zed.close();
  pclose(stream_pipe);
  log("=== Done ===");
  return 0;
}
