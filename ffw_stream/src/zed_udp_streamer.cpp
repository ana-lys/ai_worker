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
#include <condition_variable>
#include <queue>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <pthread.h>

#include "zed_udp_streamer.hpp"

std::mutex cout_mutex;
std::atomic<bool> g_running(true);

struct FrameBuffer {
    std::vector<uint8_t> data;
    int64_t timestamp_ms;
};

class FrameQueue {
    std::queue<FrameBuffer> q;
    std::mutex mtx;
    std::condition_variable cv;
    static constexpr size_t MAX_DEPTH = 2; // drop policy

public:
    void push(FrameBuffer&& frame) {
        std::unique_lock<std::mutex> lock(mtx);
        if (q.size() >= MAX_DEPTH) {
            q.pop(); // drop oldest to avoid blocking grab thread
        }
        q.push(std::move(frame));
        cv.notify_one();
    }

    FrameBuffer pop() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !q.empty() || !g_running; });
        if (!g_running && q.empty()) return {};
        auto frame = std::move(q.front());
        q.pop();
        return frame;
    }
};

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

  log("Starting threads...");
  
  FrameQueue queue;
  
  // Thread B: Pipe thread (can block freely)
  std::thread pipe_thread([&]() {
    log("Pipe thread started.");
    while (g_running) {
      auto frame = queue.pop();
      if (!g_running && frame.data.empty()) break;
      
      size_t expected_size = width * height * 4;
      if (frame.data.size() != expected_size) continue;
      
      size_t written = fwrite(frame.data.data(), 1, expected_size, stream_pipe);
      if (written != expected_size) {
        log("Short write to ffmpeg pipe, stopping");
        g_running = false;
        break;
      }
      fflush(stream_pipe);
    }
    log("Pipe thread exiting.");
  });

  // Thread A: Grab thread (high priority)
  int frame_count = 0;
  
  // Set SCHED_FIFO if possible
  struct sched_param sp;
  sp.sched_priority = 80;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
      log("Warning: Failed to set SCHED_FIFO for grab thread. Ensure you have permissions.");
  } else {
      log("Grab thread running with SCHED_FIFO priority.");
  }

  sl::Mat image_left;
  while (g_running) {
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
      zed.retrieveImage(image_left, sl::VIEW::LEFT);
      
      cv::Mat cv_frame(height, width, CV_8UC4, image_left.getPtr<sl::uchar1>(), image_left.getStepBytes());
      std::string text = "FRAME " + std::to_string(frame_count);
      cv::putText(cv_frame, text, cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 0, 255), 5);
      cv::putText(cv_frame, text, cv::Point(50, height - 100), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 0, 255, 255), 5);

      FrameBuffer buf;
      size_t expected_size = width * height * 4;
      buf.data.assign(image_left.getPtr<sl::uchar1>(), image_left.getPtr<sl::uchar1>() + expected_size);
      
      queue.push(std::move(buf));
      
      frame_count++;
      if (frame_count % 150 == 0) {
        log("ZED grabbed " + std::to_string(frame_count) + " frames");
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // Wake up pipe thread to exit
  queue.push({}); 
  if (pipe_thread.joinable()) pipe_thread.join();

  log("Stopping streamer...");
  zed.close();
  pclose(stream_pipe);
  log("=== Done ===");
  return 0;
}
