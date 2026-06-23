#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>
#include <atomic>

class RealsenseUDPReceiver : public rclcpp::Node {
public:
  RealsenseUDPReceiver() : Node("realsense_udp_receiver") {
    this->declare_parameter<int>("base_port", 9000);
    this->declare_parameter<int>("num_cameras", 2);
    this->declare_parameter<bool>("headless", false);
    
    int base_port = this->get_parameter("base_port").as_int();
    num_cameras_ = this->get_parameter("num_cameras").as_int();

    RCLCPP_INFO(this->get_logger(), "Starting Multi-Camera UDP Receiver (base_port=%d, num_cameras=%d)", base_port, num_cameras_);

    for (int i = 0; i < num_cameras_; ++i) {
      std::string ns = "camera_" + std::to_string(i);
      
      pub_depth_[i] = this->create_publisher<sensor_msgs::msg::Image>(ns + "/depth/image_rect_raw", 10);
      pub_ir_[i] = this->create_publisher<sensor_msgs::msg::Image>(ns + "/infra1/image_rect_raw", 10);

      int depth_port = base_port + (i * 2);
      int ir_port = depth_port + 1;

      threads_.emplace_back(&RealsenseUDPReceiver::streamLoop, this, i, "Depth", depth_port, pub_depth_[i]);
      threads_.emplace_back(&RealsenseUDPReceiver::streamLoop, this, i, "IR", ir_port, pub_ir_[i]);
    }

    if (!this->get_parameter("headless").as_bool()) {
      display_thread_ = std::thread(&RealsenseUDPReceiver::displayLoop, this);
    } else {
      RCLCPP_INFO(this->get_logger(), "Running in HEADLESS mode (No OpenCV Windows)");
    }
  }

  ~RealsenseUDPReceiver() {
    running_ = false;
    for (auto &t : threads_) {
      if (t.joinable()) t.join();
    }
    if (display_thread_.joinable()) display_thread_.join();

    cv::destroyAllWindows();
  }

private:
  void streamLoop(int cam_index, const std::string& type, int port, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub) {
    std::string pipeline = "udpsrc port=" + std::to_string(port) + 
      " caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264\" ! "
      "rtph264depay ! decodebin ! videoconvert ! appsink drop=true sync=false";
    
    std::string feed_name = "Cam" + std::to_string(cam_index) + "_" + type;
    
    RCLCPP_INFO(this->get_logger(), "[%s] Starting receiver on %s", feed_name.c_str(), pipeline.c_str());

    while (running_ && rclcpp::ok()) {
      cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
      
      if (!cap.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "[%s] Failed to open UDP stream, retrying in 2s...", feed_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "[%s] Successfully connected to %s", feed_name.c_str(), pipeline.c_str());

      cv::Mat frame;
      std_msgs::msg::Header header;
      header.frame_id = "camera_" + std::to_string(cam_index) + "_link";

      while (running_ && rclcpp::ok()) {
        if (!cap.read(frame) || frame.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }

        header.stamp = this->now();

        // Convert format to grayscale (H264 decodes as BGR/YUV)
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        // Rotate 90 degrees counter-clockwise as requested
        cv::rotate(gray, gray, cv::ROTATE_90_COUNTERCLOCKWISE);
        
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
        pub->publish(*msg);

        // Save for display loop
        {
          std::lock_guard<std::mutex> lock(img_mutex_);
          latest_images_[feed_name] = gray.clone();
        }

        // Calculate FPS
        frames_received_[feed_name]++;
        auto now = std::chrono::steady_clock::now();
        if (frames_received_[feed_name] == 30) {
          double elapsed = std::chrono::duration<double>(now - fps_timers_[feed_name]).count();
          double fps_val = (elapsed > 0.0) ? (30.0 / elapsed) : 0.0;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "[%s] FPS: %.1f", feed_name.c_str(), fps_val);
          frames_received_[feed_name] = 0;
          fps_timers_[feed_name] = now;
        }
      }
    }
  }

  void displayLoop() {
    std::string window_name = "Multi-Camera UDP Dashboard";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    while (running_ && rclcpp::ok()) {
      std::vector<cv::Mat> camera_grids;

      for (int i = 0; i < num_cameras_; ++i) {
        cv::Mat depth, ir;
        {
          std::lock_guard<std::mutex> lock(img_mutex_);
          std::string depth_key = "Cam" + std::to_string(i) + "_Depth";
          std::string ir_key = "Cam" + std::to_string(i) + "_IR";
          
          if (latest_images_.count(depth_key)) depth = latest_images_[depth_key].clone();
          if (latest_images_.count(ir_key)) ir = latest_images_[ir_key].clone();
        }

        // Convert to BGR for display
        if (!depth.empty() && depth.channels() == 1) {
          static cv::Mat custom_lut;
          if (custom_lut.empty()) {
            custom_lut = cv::Mat(256, 1, CV_8UC3);
            custom_lut.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0); // Invalid depth
            
            // Generate smooth cool background with a sharp red highlight
            for (int i = 1; i <= 255; ++i) { 
              float x = i / 255.0f; // distance in meters (0 to 1.0)
              
              // Black out the really near range to hide self/gripper noise
              if (x < 0.08f) {
                custom_lut.at<cv::Vec3b>(i, 0) = cv::Vec3b(0, 0, 0);
                continue;
              }
              
              // Base cool gradient: Cyan (close) to Dark Blue (far)
              float base_b = 255.0f - (127.0f * x);
              float base_g = 255.0f * (1.0f - x);
              float base_r = 0.0f;
              
              // Sharp linear "spike" exactly at 0.14m (spread of 0.01m, strictly bounding the highlight between 0.13m and 0.15m)
              float diff = x - 0.14f;
              float c = 1.0f - std::abs(diff / 0.01f);
              if (c < 0.0f) c = 0.0f; // clamp to 0 outside the critical range
              
              // Blend base color with Bright Red (0, 0, 255 in BGR) based on criticality
              float b = base_b * (1.0f - c) + 0.0f * c;
              float g = base_g * (1.0f - c) + 0.0f * c;
              float r = base_r * (1.0f - c) + 255.0f * c;
              
              custom_lut.at<cv::Vec3b>(i, 0) = cv::Vec3b(b, g, r);
            }
          }
          cv::applyColorMap(depth, depth, custom_lut);
        }
        if (!ir.empty() && ir.channels() == 1) {
          cv::cvtColor(ir, ir, cv::COLOR_GRAY2BGR);
        }

        // Ensure consistent sizes before concatenating
        // Default rotated dimensions are w=270, h=480
        int w = 270, h = 480; 
        if (!depth.empty()) { w = depth.cols; h = depth.rows; }
        else if (!ir.empty()) { w = ir.cols; h = ir.rows; }

        cv::Mat blank = cv::Mat::zeros(h, w, CV_8UC3);
        if (depth.empty()) depth = blank.clone();
        if (ir.empty()) ir = blank.clone();

        // Label images
        cv::putText(depth, "Cam " + std::to_string(i) + " Depth", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        cv::putText(ir, "Cam " + std::to_string(i) + " IR", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        // Stack IR and Depth horizontally for this camera
        cv::Mat cam_grid;
        cv::vconcat(ir, depth, cam_grid);
        camera_grids.push_back(cam_grid);
      }

      // Stack all camera columns horizontally
      if (!camera_grids.empty()) {
        cv::Mat final_grid = camera_grids[0];
        for (size_t i = 1; i < camera_grids.size(); ++i) {
          cv::hconcat(final_grid, camera_grids[i], final_grid);
        }
        cv::imshow(window_name, final_grid);
      }

      int key = cv::waitKey(15);
      if (key == 27 || key == 'q') {
        rclcpp::shutdown();
      }
    }
  }

  int num_cameras_;
  
  std::map<int, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub_depth_;
  std::map<int, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub_ir_;

  std::mutex img_mutex_;
  std::map<std::string, cv::Mat> latest_images_;
  std::map<std::string, int> frames_received_;
  std::map<std::string, std::chrono::steady_clock::time_point> fps_timers_;

  std::vector<std::thread> threads_;
  std::thread display_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseUDPReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}