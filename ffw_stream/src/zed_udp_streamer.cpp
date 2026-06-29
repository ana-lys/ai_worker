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
#include <queue>
#include <condition_variable>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <array>
#include <pthread.h>

#include "zed_udp_streamer.hpp"

std::atomic<bool> g_running(true);

class FrameQueue {
    std::queue<int> q;
    std::mutex mtx;
    std::condition_variable cv;
    static constexpr size_t MAX_DEPTH = 2; // drop policy

public:
    void push(int slot) {
        std::unique_lock<std::mutex> lock(mtx);
        if (q.size() >= MAX_DEPTH) {
            q.pop(); // drop oldest to avoid blocking grab thread
        }
        q.push(slot);
        cv.notify_one();
    }

    int pop() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !q.empty() || !g_running; });
        if (!g_running && q.empty()) return -1;
        int slot = q.front();
        q.pop();
        return slot;
    }
};

void signalHandler(int signum) {
  g_running = false;
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
}

std::mutex cout_mutex;

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

  int width = (res == 1080) ? 3840 : 2560; // Side-by-side doubles the width
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

  log("Starting Zero-Copy Decoupled streaming loop...");
  
  constexpr int POOL_SIZE = 3;
  std::array<sl::Mat, POOL_SIZE> frame_pool;
  for (auto& m : frame_pool) {
      m.alloc(width, height, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  }

  FrameQueue queue;
  std::atomic<int> write_idx{0};

  // Thread B: Pipe thread (can block freely)
  std::thread pipe_thread([&]() {
    log("Pipe thread started.");
    size_t expected_size = width * height * 4;
    while (g_running) {
      int slot = queue.pop();
      if (slot == -1) break; // exit signaled
      
      cv::Mat cv_frame(height, width, CV_8UC4, frame_pool[slot].getPtr<sl::uchar1>(), frame_pool[slot].getStepBytes());
      
      // Draw diagnostic text LEFT and RIGHT on the correct halves
      static int display_count = 0;
      std::string l_text = "LEFT " + std::to_string(display_count);
      std::string r_text = "RIGHT " + std::to_string(display_count);
      cv::putText(cv_frame, l_text, cv::Point(50, height / 2), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 0, 255), 5);
      cv::putText(cv_frame, r_text, cv::Point((width / 2) + 50, height / 2), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 0, 255, 255), 5);
      display_count++;

      // Algorithmic Vertical Tear Detector
      cv::Mat small;
      cv::resize(cv_frame, small, cv::Size(128, 64)); 
      cv::cvtColor(small, small, cv::COLOR_BGRA2GRAY);

      int best_x = -1;
      double max_diff = 0;
      
      // Search middle 50%
      for(int x = small.cols/4; x < 3*small.cols/4; ++x) {
          cv::Mat diff;
          cv::absdiff(small.col(x), small.col(x-1), diff);
          double d = cv::mean(diff)[0];
          if (d > max_diff) {
              max_diff = d;
              best_x = x;
          }
      }

      uint8_t* pipe_data = frame_pool[slot].getPtr<sl::uchar1>();
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

      size_t written = fwrite(pipe_data, 1, expected_size, stream_pipe);
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
  
  // Enforce SCHED_FIFO
  struct sched_param sp;
  sp.sched_priority = 80;
  int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (ret != 0) {
      fprintf(stderr, "FATAL: pthread_setschedparam failed: %s\n", strerror(ret));
      fprintf(stderr, "Run: sudo setcap cap_sys_nice+eip <executable>\n");
      g_running = false;
      queue.push(-1);
      if (pipe_thread.joinable()) pipe_thread.join();
      std::exit(1);
  } else {
      log("Grab thread successfully elevated to SCHED_FIFO priority 80.");
  }
  
  while (g_running) {
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
      int slot = write_idx.load() % POOL_SIZE;
      
      zed.retrieveImage(frame_pool[slot], sl::VIEW::SIDE_BY_SIDE, sl::MEM::CPU);
      
      queue.push(slot);
      write_idx++;
      
      frame_count++;
      if (frame_count % 150 == 0) {
        log("ZED grabbed " + std::to_string(frame_count) + " frames");
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  queue.push(-1); 
  if (pipe_thread.joinable()) pipe_thread.join();

  log("Stopping streamer...");
  zed.close();
  pclose(stream_pipe);
  log("=== Done ===");
  return 0;
}
