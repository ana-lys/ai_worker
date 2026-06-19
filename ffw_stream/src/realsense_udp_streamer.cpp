#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

struct FFmpegPipe {
    FILE* file = nullptr;
    std::string name;

    ~FFmpegPipe() {
        if (file) pclose(file);
    }
};

rs2_format getFormat(const std::string &fmt) {
  if (fmt == "rgb8")
    return RS2_FORMAT_RGB8;
  if (fmt == "bgr8")
    return RS2_FORMAT_BGR8;
  if (fmt == "z16")
    return RS2_FORMAT_Z16;
  if (fmt == "y8")
    return RS2_FORMAT_Y8;
  if (fmt == "y16")
    return RS2_FORMAT_Y16;
  return RS2_FORMAT_ANY;
}

class RealsenseUDPStreamer : public rclcpp::Node {
public:
  RealsenseUDPStreamer() : Node("realsense_udp_streamer") {
    std::cout << "[DEBUG] RealsenseUDPStreamer constructor START" << std::endl;

    std::cout << "[DEBUG] Creating rs2::pipeline..." << std::endl;
    pipe_ = std::make_shared<rs2::pipeline>();
    std::cout << "[DEBUG] rs2::pipeline created." << std::endl;

    std::cout << "[DEBUG] Declaring parameters..." << std::endl;
    this->declare_parameter<std::string>("target_ip", "127.0.0.1");
    this->declare_parameter<int>("target_port_rgb", 8080);
    this->declare_parameter<int>("target_port_depth", 8082);
    this->declare_parameter<int>("target_port_ir", 8084);
    this->declare_parameter<std::string>("device_id", "");
    this->declare_parameter<bool>("enable_rgb", true);
    this->declare_parameter<bool>("enable_depth", true);
    this->declare_parameter<bool>("enable_ir", false);
    this->declare_parameter<int>("rgb_width", 640);
    this->declare_parameter<int>("rgb_height", 480);
    this->declare_parameter<int>("rgb_fps", 30);
    this->declare_parameter<std::string>("rgb_format", "rgb8");
    this->declare_parameter<int>("depth_width", 640);
    this->declare_parameter<int>("depth_height", 480);
    this->declare_parameter<int>("depth_fps", 30);
    this->declare_parameter<std::string>("depth_format", "z16");
    this->declare_parameter<int>("ir_width", 640);
    this->declare_parameter<int>("ir_height", 480);
    this->declare_parameter<int>("ir_fps", 30);
    this->declare_parameter<std::string>("ir_format", "y8");
    this->declare_parameter<int>("ir_index", 1);
    this->declare_parameter<int>("target_port_metadata", 8089);

    std::cout << "[DEBUG] Parameters declared. Getting target_ip..." << std::endl;
    target_ip_ = this->get_parameter("target_ip").as_string();
    ir_index_ = this->get_parameter("ir_index").as_int();

    std::cout << "[DEBUG] Creating metadata socket..." << std::endl;
    metadata_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&metadata_addr_, 0, sizeof(metadata_addr_));
    metadata_addr_.sin_family = AF_INET;
    metadata_addr_.sin_port = htons(this->get_parameter("target_port_metadata").as_int());
    inet_pton(AF_INET, target_ip_.c_str(), &metadata_addr_.sin_addr);

    std::cout << "[DEBUG] Calling discoverDevices()..." << std::endl;
    discoverDevices();
    std::cout << "[DEBUG] discoverDevices() finished." << std::endl;

    std::cout << "[DEBUG] Creating rs2::config..." << std::endl;
    try {
      rs2::config cfg;
      std::string device_id = this->get_parameter("device_id").as_string();
      if (!device_id.empty()) {
        std::cout << "[DEBUG] Enabling device_id: " << device_id << std::endl;
        cfg.enable_device(device_id);
      }

    if (this->get_parameter("enable_rgb").as_bool()) {
      int w = this->get_parameter("rgb_width").as_int();
      int h = this->get_parameter("rgb_height").as_int();
      int fps = this->get_parameter("rgb_fps").as_int();
      cfg.enable_stream(RS2_STREAM_COLOR, w, h, getFormat(this->get_parameter("rgb_format").as_string()), fps);
      std::string cmd = "ffmpeg -hide_banner -loglevel error -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s " + std::to_string(w) + "x" + std::to_string(h) + " -r " + std::to_string(fps) + " -i - -c:v libx264 -preset ultrafast -tune zerolatency -f rtp rtp://" + target_ip_ + ":" + std::to_string(this->get_parameter("target_port_rgb").as_int());
      initFFmpegPipe(cmd, pipe_rgb_, "RGB");
    }

    if (this->get_parameter("enable_depth").as_bool()) {
      int w = this->get_parameter("depth_width").as_int();
      int h = this->get_parameter("depth_height").as_int();
      int fps = this->get_parameter("depth_fps").as_int();
      cfg.enable_stream(RS2_STREAM_DEPTH, w, h, getFormat(this->get_parameter("depth_format").as_string()), fps);
      std::string cmd = "ffmpeg -hide_banner -loglevel error -y -f rawvideo -vcodec rawvideo -pix_fmt gray -s " + std::to_string(w) + "x" + std::to_string(h) + " -r " + std::to_string(fps) + " -i - -c:v libx264 -preset ultrafast -tune zerolatency -f rtp rtp://" + target_ip_ + ":" + std::to_string(this->get_parameter("target_port_depth").as_int());
      initFFmpegPipe(cmd, pipe_depth_, "Depth");
    }

      if (this->get_parameter("enable_ir").as_bool()) {
        std::cout << "[DEBUG] Configuring IR stream..." << std::endl;
        int w = this->get_parameter("ir_width").as_int();
        int h = this->get_parameter("ir_height").as_int();
        int fps = this->get_parameter("ir_fps").as_int();
        cfg.enable_stream(RS2_STREAM_INFRARED, ir_index_, w, h, getFormat(this->get_parameter("ir_format").as_string()), fps);
        std::string cmd = "ffmpeg -hide_banner -loglevel error -y -f rawvideo -vcodec rawvideo -pix_fmt gray -s " + std::to_string(w) + "x" + std::to_string(h) + " -r " + std::to_string(fps) + " -i - -c:v libx264 -preset ultrafast -tune zerolatency -f rtp rtp://" + target_ip_ + ":" + std::to_string(this->get_parameter("target_port_ir").as_int());
        initFFmpegPipe(cmd, pipe_ir_, "IR");
      }

      std::cout << "[DEBUG] Starting pipeline (pipe_->start(cfg))..." << std::endl;
      auto prof = pipe_->start(cfg);
      profile_ = std::make_shared<rs2::pipeline_profile>(prof);
      std::cout << "[DEBUG] Pipeline started successfully." << std::endl;
      
      auto dev = profile_->get_device();
      for (auto &sensor : dev.query_sensors()) {
        if (sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f);
        if (sensor.is<rs2::depth_sensor>()) depth_scale_ = sensor.as<rs2::depth_sensor>().get_depth_scale();
      }
    } catch (const rs2::error &e) {
      std::cout << "[DEBUG] Caught rs2::error: " << e.what() << std::endl;
      RCLCPP_ERROR(this->get_logger(), "RealSense config error: %s", e.what());
      return;
    } catch (const std::exception &e) {
      std::cout << "[DEBUG] Caught std::exception: " << e.what() << std::endl;
      return;
    } catch (...) {
      std::cout << "[DEBUG] Caught unknown exception!" << std::endl;
      return;
    }

    std::cout << "[DEBUG] Launching streamLoop thread..." << std::endl;
    running_ = true;
    stream_thread_ = std::thread(&RealsenseUDPStreamer::streamLoop, this);
    std::cout << "[DEBUG] RealsenseUDPStreamer constructor END" << std::endl;
  }

  ~RealsenseUDPStreamer() {
    running_ = false;
    if (stream_thread_.joinable()) stream_thread_.join();
    try { if (pipe_) pipe_->stop(); } catch (...) {}
    if (metadata_sock_ >= 0) close(metadata_sock_);
  }

private:
  void initFFmpegPipe(const std::string &cmd, std::shared_ptr<FFmpegPipe> &pipe, const std::string &name) {
    std::cout << "[DEBUG] initFFmpegPipe for " << name << " START" << std::endl;
    pipe = std::make_shared<FFmpegPipe>();
    pipe->name = name;
    std::cout << "[DEBUG] popen: " << cmd << std::endl;
    pipe->file = popen(cmd.c_str(), "w");
    if (!pipe->file) {
      std::cout << "[DEBUG] popen FAILED for " << name << std::endl;
      RCLCPP_ERROR(this->get_logger(), "Failed to open FFmpeg pipe for %s", name.c_str());
    } else {
      std::cout << "[DEBUG] popen SUCCESS for " << name << std::endl;
    }
  }

  void pushFFmpegBuffer(const rs2::video_frame &frame, std::shared_ptr<FFmpegPipe> &pipe, const std::string &stream_name, uint32_t frame_id, double timestamp, bool is_depth = false) {
    if (!frame || !pipe || !pipe->file) return;
    if (is_depth) {
      const uint16_t* depth_data = (const uint16_t*)frame.get_data();
      int pixels = frame.get_width() * frame.get_height();
      std::vector<uint8_t> gray8(pixels);
      float max_val = 1.0f / depth_scale_;
      for (int i = 0; i < pixels; ++i) {
        float scaled = (float)depth_data[i] / max_val;
        gray8[i] = (uint8_t)(std::min(scaled, 1.0f) * 255.0f);
      }
      fwrite(gray8.data(), 1, pixels, pipe->file);
    } else {
      fwrite(frame.get_data(), 1, frame.get_width() * frame.get_height() * frame.get_bytes_per_pixel(), pipe->file);
    }
    char msg[256];
    int len = snprintf(msg, sizeof(msg), "{\"stream\":\"%s\",\"frame_id\":%u,\"timestamp\":%f}", stream_name.c_str(), frame_id, timestamp);
    sendto(metadata_sock_, msg, len, 0, (struct sockaddr *)&metadata_addr_, sizeof(metadata_addr_));
  }

  void discoverDevices() {
    try {
      rs2::context ctx;
      auto devices = ctx.query_devices();
      RCLCPP_INFO(this->get_logger(), "Found %zu RealSense device(s)", (size_t)devices.size());
    } catch (const rs2::error &e) {
      RCLCPP_WARN(this->get_logger(), "Could not discover devices: %s", e.what());
    }
  }

  void streamLoop() {
    uint32_t frame_id = 0;
    while (running_ && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        if (!pipe_) continue;
        frames = pipe_->wait_for_frames(5000);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Timeout waiting for frames: %s", e.what());
        continue;
      }
      
      double ts = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      
      if (pipe_rgb_) {
        pushFFmpegBuffer(frames.get_color_frame(), pipe_rgb_, "RGB", frame_id, ts);
      }
      if (pipe_depth_) {
        pushFFmpegBuffer(frames.get_depth_frame(), pipe_depth_, "Depth", frame_id, ts, true);
      }
      if (pipe_ir_) {
        pushFFmpegBuffer(frames.get_infrared_frame(ir_index_), pipe_ir_, "IR", frame_id, ts);
      }
      
      frame_id++;
    }
  }

  std::string target_ip_;
  int ir_index_ = 1;
  std::shared_ptr<FFmpegPipe> pipe_rgb_;
  std::shared_ptr<FFmpegPipe> pipe_depth_;
  std::shared_ptr<FFmpegPipe> pipe_ir_;
  std::shared_ptr<rs2::pipeline> pipe_;
  std::shared_ptr<rs2::pipeline_profile> profile_;
  std::thread stream_thread_;
  std::atomic<bool> running_{false};
  int metadata_sock_ = -1;
  struct sockaddr_in metadata_addr_;
  float depth_scale_ = 0.001f;
};

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
  std::cout << "[DEBUG] main START" << std::endl;
  rclcpp::init(argc, argv);
  std::cout << "[DEBUG] rclcpp::init DONE. Creating dummy node..." << std::endl;
  
  try {
    auto dummy = std::make_shared<rclcpp::Node>("dummy_test_node");
    std::cout << "[DEBUG] Dummy node created successfully!" << std::endl;
  } catch (const std::exception &e) {
    std::cout << "[DEBUG] Dummy node THREW EXCEPTION: " << e.what() << std::endl;
  }

  std::cout << "[DEBUG] Creating actual RealsenseUDPStreamer node..." << std::endl;
  auto node = std::make_shared<RealsenseUDPStreamer>();
  std::cout << "[DEBUG] Node created. Spinning..." << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
  std::cout << "[DEBUG] main END" << std::endl;
  return 0;
}