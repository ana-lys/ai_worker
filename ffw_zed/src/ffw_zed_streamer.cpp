#include <atomic>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <sl/Camera.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <array>
#include <pthread.h>

std::mutex cout_mutex;
std::atomic<bool> g_running(true);

class FrameQueue {
    std::queue<int> q;
    std::mutex mtx;
    std::condition_variable cv;
    static constexpr size_t MAX_DEPTH = 2; 

public:
    void push(int slot) {
        std::unique_lock<std::mutex> lock(mtx);
        if (q.size() >= MAX_DEPTH) {
            q.pop(); 
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

void log(const std::string &msg) {
  std::lock_guard<std::mutex> lock(cout_mutex);
  std::cout << msg << std::endl;
}

extern "C" int start_stream(int argc, char **argv) {
  (void)argc;
  (void)argv;
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  sl::Camera zed;
  sl::InitParameters init_params;
  
  // MATCHING zed-ros2-wrapper INITIALIZATION EXACTLY
  init_params.camera_resolution = sl::RESOLUTION::HD1080;
  init_params.camera_fps = 30;
  init_params.depth_mode = sl::DEPTH_MODE::NONE;
  init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
  init_params.coordinate_units = sl::UNIT::METER;
  init_params.sdk_verbose = false;
  init_params.depth_stabilization = 0;
  init_params.camera_image_flip = sl::FLIP_MODE::OFF;
  init_params.camera_disable_self_calib = false;
  init_params.enable_image_enhancement = true;
  init_params.enable_right_side_measure = false;
  init_params.async_image_retrieval = false;
  init_params.enable_image_validity_check = false;
  init_params.async_grab_camera_recovery = true;

  sl::ERROR_CODE err = zed.open(init_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    log("Failed to open ZED camera: " + std::string(sl::toString(err).c_str()));
    return 1;
  }

  int width = zed.getCameraInformation().camera_configuration.resolution.width;
  int height = zed.getCameraInformation().camera_configuration.resolution.height;
  log("ZED Camera opened. Resolution: " + std::to_string(width) + "x" + std::to_string(height));

  std::string ffmpeg_cmd = 
      "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt bgra "
      "-s " + std::to_string(width) + "x" + std::to_string(height) + " -r 30 "
      "-i - -c:v libx264 -preset ultrafast -tune zerolatency "
      "-b:v 8M -minrate 8M -maxrate 8M -bufsize 8M "
      "-x264opts keyint=30:min-keyint=30:slice-max-size=1200 -f mpegts "
      "udp://192.168.1.185:5000?pkt_size=1316";

  FILE *stream_pipe = popen(ffmpeg_cmd.c_str(), "w");
  if (!stream_pipe) {
    log("Failed to open pipe to ffmpeg");
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

  std::thread pipe_thread([&]() {
    log("Pipe thread started.");
    size_t expected_size = width * height * 4;
    while (g_running) {
      int slot = queue.pop();
      if (slot == -1) break;
      
      size_t written = fwrite(frame_pool[slot].getPtr<sl::uchar1>(), 1, expected_size, stream_pipe);
      if (written != expected_size) {
        log("Short write to ffmpeg pipe, stopping");
        g_running = false;
        break;
      }
      fflush(stream_pipe);
    }
    log("Pipe thread exiting.");
  });

  int frame_count = 0;
  
  // Enforce SCHED_FIFO EXACTLY as zed-ros2-wrapper does it
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
  
  // MATCHING zed-ros2-wrapper RUNTIME PARAMETERS EXACTLY
  sl::RuntimeParameters mRunParams;
  mRunParams.enable_depth = false;
  mRunParams.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
  mRunParams.remove_saturated_areas = true; 

  while (g_running) {
    sl::ERROR_CODE grab_status = zed.grab(mRunParams);
    
    if (grab_status == sl::ERROR_CODE::SUCCESS) {
      int slot = write_idx.load() % POOL_SIZE;
      zed.retrieveImage(frame_pool[slot], sl::VIEW::LEFT, sl::MEM::CPU);
      queue.push(slot);
      write_idx++;
      
      frame_count++;
      if (frame_count % 150 == 0) {
        log("ZED grabbed " + std::to_string(frame_count) + " frames");
      }
    } else if (grab_status == sl::ERROR_CODE::CORRUPTED_FRAME) {
      log("ZED SDK REPORTED CORRUPTED FRAME! Dropping...");
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  queue.push(-1); 
  if (pipe_thread.joinable()) pipe_thread.join();

  log("Stopping streamer...");
  zed.close();
  pclose(stream_pipe);
  return 0;
}
