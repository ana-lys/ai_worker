#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>

class RealsenseUDPReceiver : public rclcpp::Node {
public:
  RealsenseUDPReceiver() : Node("realsense_udp_receiver") {
    // Ports
    this->declare_parameter<int>("target_port_rgb", 9002);
    this->declare_parameter<int>("target_port_depth", 9000);
    this->declare_parameter<int>("target_port_ir", 9001);
    this->declare_parameter<int>("target_port_metadata", 9009);

    // Camera identity for display namespaces
    this->declare_parameter<std::string>("camera_name", "Camera");
    camera_name_ = this->get_parameter("camera_name").as_string();

    this->declare_parameter<bool>("headless", false);

    // Streams enabled
    this->declare_parameter<bool>("enable_rgb", true);
    this->declare_parameter<bool>("enable_depth", true);
    this->declare_parameter<bool>("enable_ir", false);

    // Publishers
    pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);
    pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_rect_raw", 10);
    pub_ir_ = this->create_publisher<sensor_msgs::msg::Image>("camera/infra1/image_rect_raw", 10);

    RCLCPP_INFO(this->get_logger(), "C++ FFMPEG UDP Receiver Initialized.");

    metadata_thread_ = std::thread(&RealsenseUDPReceiver::metadataLoop, this);

    if (this->get_parameter("enable_rgb").as_bool()) {
      int port = this->get_parameter("target_port_rgb").as_int();
      rgb_thread_ = std::thread(&RealsenseUDPReceiver::streamLoop, this, "RGB", port);
    }

    if (this->get_parameter("enable_depth").as_bool()) {
      int port = this->get_parameter("target_port_depth").as_int();
      depth_thread_ = std::thread(&RealsenseUDPReceiver::streamLoop, this, "Depth", port);
    }

    if (this->get_parameter("enable_ir").as_bool()) {
      int port = this->get_parameter("target_port_ir").as_int();
      ir_thread_ = std::thread(&RealsenseUDPReceiver::streamLoop, this, "IR", port);
    }

    if (!this->get_parameter("headless").as_bool()) {
      display_thread_ = std::thread(&RealsenseUDPReceiver::displayLoop, this);
    } else {
      RCLCPP_INFO(this->get_logger(), "Running in HEADLESS mode (No OpenCV Windows)");
    }
  }

  ~RealsenseUDPReceiver() {
    running_ = false;
    if (rgb_thread_.joinable()) rgb_thread_.join();
    if (depth_thread_.joinable()) depth_thread_.join();
    if (ir_thread_.joinable()) ir_thread_.join();
    if (display_thread_.joinable()) display_thread_.join();
    if (metadata_thread_.joinable()) metadata_thread_.join();

    cv::destroyAllWindows();
  }

private:
  void streamLoop(const std::string& name, int port) {
    std::string pipeline = "udp://@0.0.0.0:" + std::to_string(port);
    
    RCLCPP_INFO(this->get_logger(), "[%s] Starting receiver on %s", name.c_str(), pipeline.c_str());

    while (running_ && rclcpp::ok()) {
      cv::VideoCapture cap(pipeline, cv::CAP_FFMPEG);
      
      if (!cap.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "[%s] Failed to open UDP stream, retrying in 2s...", name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "[%s] Successfully connected to %s", name.c_str(), pipeline.c_str());

      cv::Mat frame;
      std_msgs::msg::Header header;
      header.frame_id = "camera_link";

      while (running_ && rclcpp::ok()) {
        if (!cap.read(frame) || frame.empty()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "[%s] Frame empty or stream dropped, attempting to read...", name.c_str());
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }

        header.stamp = this->now();

        // Convert format and publish
        cv::Mat display_frame;
        if (name == "RGB") {
          display_frame = frame.clone();
          sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
          pub_rgb_->publish(*msg);
        } else if (name == "IR") {
          cv::Mat gray;
          cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
          display_frame = gray.clone();
          sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
          pub_ir_->publish(*msg);
        } else if (name == "Depth") {
          cv::Mat gray;
          cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
          display_frame = gray.clone();
          sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
          pub_depth_->publish(*msg);
        }

        // Save for display loop
        {
          std::lock_guard<std::mutex> lock(img_mutex_);
          latest_images_[name] = display_frame;
        }

        // Calculate FPS
        frames_received_[name]++;
        auto now = std::chrono::steady_clock::now();
        if (frames_received_[name] == 30) {
          double elapsed = std::chrono::duration<double>(now - fps_timers_[name]).count();
          double fps_val = (elapsed > 0.0) ? (30.0 / elapsed) : 0.0;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "[%s] Receiver FPS: %.1f", name.c_str(), fps_val);
          frames_received_[name] = 0;
          fps_timers_[name] = now;
        }
      }
    }
  }

  void displayLoop() {
    std::string window_name = camera_name_ + " Feeds";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    while (running_ && rclcpp::ok()) {
      cv::Mat rgb, depth, ir;
      {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (latest_images_.count("RGB")) rgb = latest_images_["RGB"].clone();
        if (latest_images_.count("Depth")) depth = latest_images_["Depth"].clone();
        if (latest_images_.count("IR")) ir = latest_images_["IR"].clone();
      }

      // Convert grayscale images to BGR for concatenation
      if (!depth.empty() && depth.channels() == 1) {
        cv::applyColorMap(depth, depth, cv::COLORMAP_JET);
      }
      if (!ir.empty() && ir.channels() == 1) {
        cv::cvtColor(ir, ir, cv::COLOR_GRAY2BGR);
      }

      // Ensure consistent sizes before concatenating
      int w = 480, h = 270; 
      if (!rgb.empty()) { w = rgb.cols; h = rgb.rows; }
      else if (!depth.empty()) { w = depth.cols; h = depth.rows; }

      cv::Mat blank = cv::Mat::zeros(h, w, CV_8UC3);
      if (rgb.empty()) rgb = blank.clone();
      if (depth.empty()) depth = blank.clone();
      if (ir.empty()) ir = blank.clone();

      // We have up to 3 feeds. Let's arrange them in a 2x2 grid.
      // Top: RGB and Depth
      // Bottom: IR and Blank (or nothing if IR is disabled)
      cv::Mat top_row, bottom_row, grid;
      cv::hconcat(rgb, depth, top_row);

      if (this->get_parameter("enable_ir").as_bool()) {
        cv::hconcat(ir, blank, bottom_row);
        cv::vconcat(top_row, bottom_row, grid);
      } else {
        grid = top_row;
      }

      if (!grid.empty()) {
        cv::imshow(window_name, grid);
      }

      int key = cv::waitKey(15);
      if (key == 27 || key == 'q') {
        rclcpp::shutdown();
      }
    }
  }

  void metadataLoop() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create metadata UDP socket");
      return;
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(this->get_parameter("target_port_metadata").as_int());

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind metadata UDP socket");
      close(sock);
      return;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char buffer[512];
    while (running_ && rclcpp::ok()) {
      int len = recv(sock, buffer, sizeof(buffer) - 1, 0);
      if (len > 0) {
        // Keep the UDP socket open and draining so the buffer doesn't fill up,
        // but we don't strictly need to parse it for H264 since OpenCV handles timing.
      }
    }
    close(sock);
  }

  std::string camera_name_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_;

  std::mutex img_mutex_;
  std::map<std::string, cv::Mat> latest_images_;
  std::map<std::string, int> frames_received_;
  std::map<std::string, std::chrono::steady_clock::time_point> fps_timers_;

  std::thread rgb_thread_;
  std::thread depth_thread_;
  std::thread ir_thread_;
  std::thread display_thread_;
  std::thread metadata_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseUDPReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}