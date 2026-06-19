#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <thread>

class RealsenseStreamer : public rclcpp::Node {
public:
  RealsenseStreamer() : Node("realsense_streamer") {
    this->declare_parameter<std::string>("device_id", "");

    device_id_ = this->get_parameter("device_id").as_string();

    if (device_id_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "device_id parameter is required!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Opening camera: %s", device_id_.c_str());

    try {
      rs2::config cfg;
      cfg.enable_device(device_id_);
      cfg.enable_stream(RS2_STREAM_DEPTH,     480, 270, RS2_FORMAT_Z16, 30);
      cfg.enable_stream(RS2_STREAM_INFRARED, 1, 480, 270, RS2_FORMAT_Y8, 30);

      profile_ = pipe_.start(cfg);
      running_ = true;

      RCLCPP_INFO(this->get_logger(), "Pipeline started OK for device: %s", device_id_.c_str());

      stream_thread_ = std::thread(&RealsenseStreamer::streamLoop, this);

    } catch (const rs2::error& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline: %s", e.what());
    }
  }

  ~RealsenseStreamer() {
    running_ = false;
    if (stream_thread_.joinable()) stream_thread_.join();
    try { pipe_.stop(); } catch (...) {}
  }

private:
  void streamLoop() {
    uint32_t frame_count = 0;
    auto last_print = std::chrono::steady_clock::now();

    while (running_ && rclcpp::ok()) {
      try {
        rs2::frameset frames = pipe_.wait_for_frames(5000);
        if (frames) frame_count++;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Frame timeout: %s", e.what());
        continue;
      }

      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 2) {
        RCLCPP_INFO(this->get_logger(), "Receiving frames: %u in last 2s", frame_count);
        frame_count = 0;
        last_print = now;
      }
    }
  }

  std::string device_id_;
  rs2::pipeline pipe_;
  rs2::pipeline_profile profile_;
  std::thread stream_thread_;
  std::atomic<bool> running_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseStreamer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}