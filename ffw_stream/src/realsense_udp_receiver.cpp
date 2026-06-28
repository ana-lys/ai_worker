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
#include <ctime>
#include <cstdlib>

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

    int zed_port = base_port + 100;
    threads_.emplace_back(&RealsenseUDPReceiver::zedStreamLoop, this, zed_port);
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
      " buffer-size=2147483647 caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264\" ! "
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

        // Calculate FPS (60-second average)
        frames_received_[feed_name]++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_timers_[feed_name]).count();
        if (elapsed >= 60.0) {
          double fps_val = (elapsed > 0.0) ? (frames_received_[feed_name] / elapsed) : 0.0;
          if (fps_val < 28.0) {
            RCLCPP_WARN(this->get_logger(), "[%s] Frame drop detected! Average FPS: %.1f", feed_name.c_str(), fps_val);
          }
          frames_received_[feed_name] = 0;
          fps_timers_[feed_name] = now;
        }
      }
    }
  }

  void zedStreamLoop(int port) {
    std::string pipeline = "udpsrc port=" + std::to_string(port) + 
      " buffer-size=2147483647 caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264\" ! "
      "rtph264depay ! decodebin ! videoconvert ! appsink drop=true sync=false";
    
    std::string feed_name = "ZED";
    RCLCPP_INFO(this->get_logger(), "[%s] Starting receiver on %s", feed_name.c_str(), pipeline.c_str());

    while (running_ && rclcpp::ok()) {
      cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
      if (!cap.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "[%s] Failed to open UDP stream, retrying in 2s...", feed_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "[%s] Successfully connected", feed_name.c_str());

      cv::Mat frame;
      cv::Mat prev_frame;
      cv::Mat prev_gray; // ADDED for optimization
      int glitch_counter = 0;
      std::string record_dir;

      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      char buf[64];
      std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
      
      const char* home_dir = getenv("HOME");
      if (home_dir) {
        record_dir = std::string(home_dir) + "/record/glitches/" + buf + "/";
        std::string cmd = "mkdir -p " + record_dir;
        system(cmd.c_str());
        RCLCPP_INFO(this->get_logger(), "[ZED] Glitch detector active. Saving to: %s", record_dir.c_str());
      }

      while (running_ && rclcpp::ok()) {
        if (!cap.read(frame) || frame.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }

        // --- ULTRA FAST GLITCH DETECTOR LOGIC ---
        if (!record_dir.empty()) {
          cv::Mat gray_curr;
          // Downsample massively for speed (160x90)
          cv::resize(frame, gray_curr, cv::Size(160, 90));
          cv::cvtColor(gray_curr, gray_curr, cv::COLOR_BGR2GRAY);
          
          if (!prev_gray.empty()) {
            cv::Mat diff;
            cv::absdiff(gray_curr, prev_gray, diff);
            double mad = cv::mean(diff)[0];
            
            // Lowered threshold to 20.0 since we removed blur
            if (mad > 20.0) {
               RCLCPP_WARN(this->get_logger(), "[ZED] GLITCH DETECTED! MAD = %.2f. Saving images...", mad);
               char fn_buf[128];
               
               sprintf(fn_buf, "glitch_%04d_A_prev.jpg", glitch_counter);
               cv::imwrite(record_dir + fn_buf, prev_frame);
               
               sprintf(fn_buf, "glitch_%04d_B_post.jpg", glitch_counter);
               cv::imwrite(record_dir + fn_buf, frame);
               
               cv::Mat diff_color;
               cv::applyColorMap(diff, diff_color, cv::COLORMAP_JET);
               // Scale up diff map for easier viewing
               cv::resize(diff_color, diff_color, cv::Size(1280, 720), 0, 0, cv::INTER_NEAREST);
               sprintf(fn_buf, "glitch_%04d_D_diff.jpg", glitch_counter);
               cv::imwrite(record_dir + fn_buf, diff_color);
               
               glitch_counter++;
            }
          }
          prev_gray = gray_curr.clone();
          prev_frame = frame.clone();
        }
        // -----------------------------

        {
          std::lock_guard<std::mutex> lock(img_mutex_);
          latest_images_[feed_name] = frame.clone();
        }
        
        frames_received_[feed_name]++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_timers_[feed_name]).count();
        if (elapsed >= 60.0) {
          double fps_val = (elapsed > 0.0) ? (frames_received_[feed_name] / elapsed) : 0.0;
          if (fps_val < 28.0) {
            RCLCPP_WARN(this->get_logger(), "[%s] Frame drop detected! Average FPS: %.1f", feed_name.c_str(), fps_val);
          }
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
              
              // ^4 highlight exactly at 0.135m (spread of 0.02m for a +/- 2cm full range)
              float diff = x - 0.135f;
              float c = 1.0f - std::pow(diff / 0.02f, 4.0f);
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
          ir.convertTo(ir, -1, 1.5, 30); // Boost brightness slightly (alpha=1.5, beta=30)
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

        // Fetch ZED image
        cv::Mat zed_frame;
        {
          std::lock_guard<std::mutex> lock(img_mutex_);
          if (latest_images_.count("ZED")) zed_frame = latest_images_["ZED"].clone();
        }

        if (!zed_frame.empty() && final_grid.rows > 0) {
          // Scale final_grid height to match ZED's height
          double scale = (double)zed_frame.rows / final_grid.rows;
          int new_width = std::round(final_grid.cols * scale);
          cv::Mat final_grid_scaled;
          cv::resize(final_grid, final_grid_scaled, cv::Size(new_width, zed_frame.rows));
          
          cv::putText(zed_frame, "ZED Feed", cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);
          
          cv::Mat dashboard;
          cv::hconcat(zed_frame, final_grid_scaled, dashboard);
          cv::imshow(window_name, dashboard);
        } else {
          cv::imshow(window_name, final_grid);
        }
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