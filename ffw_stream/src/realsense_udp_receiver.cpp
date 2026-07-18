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
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class RealsenseUDPReceiver : public rclcpp::Node {
public:
  RealsenseUDPReceiver() : Node("realsense_udp_receiver") {
    this->declare_parameter<int>("base_port", 9000);
    this->declare_parameter<int>("num_cameras", 2);
    this->declare_parameter<bool>("headless", false);
    this->declare_parameter<bool>("enable_d405s", true);
    this->declare_parameter<int>("depthai_video_port", 9100);
    this->declare_parameter<std::string>("rgb_source", "zedm");
    this->declare_parameter<std::string>("oakd_codec", "mjpeg"); // "mjpeg" or "h264"
    
    int base_port = this->get_parameter("base_port").as_int();
    num_cameras_ = this->get_parameter("num_cameras").as_int();
    bool enable_d405s = this->get_parameter("enable_d405s").as_bool();
    int depthai_video_port = this->get_parameter("depthai_video_port").as_int();
    rgb_source_ = this->get_parameter("rgb_source").as_string();
    oakd_codec_ = this->get_parameter("oakd_codec").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting Multi-Camera UDP Receiver (base_port=%d, num_cameras=%d)", base_port, num_cameras_);

    if (enable_d405s) {
      pub_depth_.resize(num_cameras_);
      pub_ir_.resize(num_cameras_);

      for (int i = 0; i < num_cameras_; ++i) {
        std::string ns = "camera_" + std::to_string(i);
        
        pub_depth_[i] = this->create_publisher<sensor_msgs::msg::Image>(ns + "/depth/image_rect_raw", 10);
        pub_ir_[i] = this->create_publisher<sensor_msgs::msg::Image>(ns + "/infra1/image_rect_raw", 10);

        int depth_port = base_port + (i * 2);
        int ir_port = depth_port + 1;

        threads_.emplace_back(&RealsenseUDPReceiver::streamLoop, this, i, "Depth", depth_port, pub_depth_[i]);
        threads_.emplace_back(&RealsenseUDPReceiver::streamLoop, this, i, "IR", ir_port, pub_ir_[i]);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "D405 streams disabled; only RGB/base_port+100 will be received");
    }

    bool headless = this->get_parameter("headless").as_bool();
    const char *display = std::getenv("DISPLAY");
    const char *wayland_display = std::getenv("WAYLAND_DISPLAY");
    RCLCPP_INFO(this->get_logger(), "GUI env: DISPLAY=%s WAYLAND_DISPLAY=%s",
                display ? display : "<unset>",
                wayland_display ? wayland_display : "<unset>");

    if (!headless) {
      display_thread_ = std::thread(&RealsenseUDPReceiver::displayLoop, this);
    } else {
      RCLCPP_INFO(this->get_logger(), "Running in HEADLESS mode (No OpenCV Windows)");
    }

    int zed_port = base_port + 100;
    threads_.emplace_back(&RealsenseUDPReceiver::zedStreamLoop, this, zed_port);
    
    // Realsense telemetry on base_port + 200
    int rs_telemetry_port = base_port + 200;
    threads_.emplace_back(&RealsenseUDPReceiver::telemetryReceiverLoop, this, "RS", rs_telemetry_port);

    // OAK-D (depthai) telemetry on depthai_video_port + 200 (bidirectional — RTT calibration)
    int depthai_telemetry_port = depthai_video_port + 200;
    threads_.emplace_back(&RealsenseUDPReceiver::oakdTelemetryLoop, this, depthai_telemetry_port);
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
      "rtpjitterbuffer latency=10 ! rtph264depay ! decodebin ! videoconvert ! appsink drop=true sync=false";
    
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

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::rotate(gray, gray, cv::ROTATE_90_COUNTERCLOCKWISE);
        
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
        pub->publish(*msg);

        {
          std::lock_guard<std::mutex> lock(img_mutex_);
          latest_images_[feed_name] = gray.clone();
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

  void telemetryReceiverLoop(const std::string &source, int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      close(sock);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[Telemetry/%s] Listening on UDP %d", source.c_str(), port);

    char buffer[1024];
    while (running_ && rclcpp::ok()) {
      int n = recv(sock, buffer, sizeof(buffer) - 1, 0);
      if (n > 0) {
        buffer[n] = '\0';
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        latest_telemetry_[source] = std::string(buffer);
      }
    }
    close(sock);
  }

  void oakdTelemetryLoop(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      close(sock);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "[Telemetry/OAK-D] Listening on UDP %d (RTT calibration enabled)", port);

    char buffer[1024];
    struct sockaddr_in sender_addr;
    socklen_t sender_len;
    bool have_sender = false;

    // Calibration state
    double cal_offset_ms = 0.0;
    auto last_cal_req = std::chrono::steady_clock::now();
    const auto cal_interval = std::chrono::seconds(30);

    while (running_ && rclcpp::ok()) {
      auto now = std::chrono::steady_clock::now();

      // ── Periodic RTT calibration ─────────────────────────────────────
      if (have_sender && (now - last_cal_req) >= cal_interval) {
        auto t1 = std::chrono::steady_clock::now();
        sendto(sock, "CAL_REQ", 7, 0,
               (struct sockaddr*)&sender_addr, sizeof(sender_addr));

        // Tight timeout for the response (200 ms)
        struct timeval cal_tv;
        cal_tv.tv_sec = 0;
        cal_tv.tv_usec = 200000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &cal_tv, sizeof(cal_tv));

        // Retry up to 5 times in case a telemetry packet arrives first
        for (int attempt = 0; attempt < 5; ++attempt) {
          struct sockaddr_in from;
          socklen_t from_len = sizeof(from);
          char resp[256];
          int n = recvfrom(sock, resp, sizeof(resp) - 1, 0,
                           (struct sockaddr*)&from, &from_len);
          if (n <= 0) break;  // timeout → give up

          resp[n] = '\0';

          // If telemetry arrived during the cal window, consume it normally
          if (strncmp(resp, "[OAK-D]", 7) == 0) {
            processOakdTelemetry(resp, cal_offset_ms);
            continue;  // try recvfrom again for CAL_RES
          }

          double oakt = -1.0;
          if (sscanf(resp, "CAL_RES oakt=%lf", &oakt) == 1 && oakt >= 0.0) {
            auto t4 = std::chrono::steady_clock::now();
            double t1_ms = std::chrono::duration<double, std::milli>(
                               t1.time_since_epoch()).count();
            double t4_ms = std::chrono::duration<double, std::milli>(
                               t4.time_since_epoch()).count();
            // offset = receiver_steady_time - oakd_hw_time
            // Use midpoint of (T1, T4) to cancel out symmetric RTT
            cal_offset_ms = ((t1_ms + t4_ms) / 2.0) - oakt;
            double rtt = t4_ms - t1_ms;
            RCLCPP_INFO(this->get_logger(),
                        "[OAK-D] Calibration: RTT=%.1fms offset=%.1fms",
                        rtt, cal_offset_ms);
            break;
          }
        }

        // Restore normal 1s timeout
        struct timeval long_tv;
        long_tv.tv_sec = 1;
        long_tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &long_tv, sizeof(long_tv));

        last_cal_req = now;
        continue;
      }

      // ── Normal telemetry receive ─────────────────────────────────────
      sender_len = sizeof(sender_addr);
      int n = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                       (struct sockaddr*)&sender_addr, &sender_len);
      if (n > 0) {
        buffer[n] = '\0';

        // Skip stray CAL_RES (shouldn't happen outside request window)
        if (strncmp(buffer, "CAL_RES", 7) == 0) continue;

        // First packet from a new sender? Save address for calibration.
        if (!have_sender) {
          have_sender = true;
          // Trigger first calibration in ~5 seconds
          last_cal_req = std::chrono::steady_clock::now() - cal_interval
                       + std::chrono::seconds(5);
          char ip_str[INET_ADDRSTRLEN];
          inet_ntop(AF_INET, &sender_addr.sin_addr, ip_str, sizeof(ip_str));
          RCLCPP_INFO(this->get_logger(),
                      "[OAK-D] Sender identified: %s:%d",
                      ip_str, ntohs(sender_addr.sin_port));
        }

        processOakdTelemetry(buffer, cal_offset_ms);
      }
    }
    close(sock);
  }

  void processOakdTelemetry(const char *buffer, double cal_offset_ms) {
    if (!buffer || !*buffer) return;

    std::lock_guard<std::mutex> lock(telemetry_mutex_);

    // Parse TW:<sender_host_ms> from the telemetry string (sender's steady_clock)
    double tw_ts = -1.0;
    const char *tw_pos = strstr(buffer, "TW:");
    if (tw_pos) {
      tw_ts = std::atof(tw_pos + 3);
    }

    if (tw_ts >= 0.0 && cal_offset_ms >= 0.0) {
      // Compute one-way latency using RTT-calibrated clock offset
      double receiver_now_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      double latency = receiver_now_ms - (tw_ts + cal_offset_ms);
      if (latency < 0.0) latency = 0.0;
      int lat_int = static_cast<int>(std::round(latency));
      last_oakd_latency_str_ = " | Latency: " + std::to_string(lat_int) + " ms";
      latest_telemetry_["OAK-D"] = std::string(buffer) + last_oakd_latency_str_;
    } else {
      // System info or uncalibrated: show the message + last known latency (if any)
      std::string enriched = buffer;
      if (!last_oakd_latency_str_.empty()) {
        enriched += last_oakd_latency_str_;
      }
      latest_telemetry_["OAK-D"] = enriched;
    }
  }

  void zedStreamLoop(int port) {
    std::string feed_name;
    std::string pipeline;

    if (rgb_source_ == "oakd_lite") {
      feed_name = "OAK-D";
      if (oakd_codec_ == "h264") {
        // H264-Baseline receiver:
        //   - latency=0 on jitter buffer: no deliberate buffering, display ASAP
        //   - h264parse: reframes the Baseline bitstream for avdec_h264
        //   - avdec_h264 max-threads=1: single-thread decode avoids thread-sync overhead
        //   - queue leaky=downstream: if decode is slow, drop old frames not new ones
        pipeline = "udpsrc port=" + std::to_string(port) +
          " buffer-size=2147483647 "
          "caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264\" ! "
          "rtpjitterbuffer latency=20 ! rtph264depay ! decodebin ! videoconvert ! "
          "queue max-size-buffers=1 leaky=downstream ! "
          "appsink drop=true sync=false async=false max-buffers=1";
        RCLCPP_INFO(this->get_logger(), "[OAK-D] Using H264-Baseline receiver pipeline");
      } else {
        // MJPEG fallback (original)
        // caps match: gst-launch-1.0 udpsrc port=9100 caps="application/x-rtp, media=video, encoding-name=JPEG, payload=26"
        pipeline = "udpsrc port=" + std::to_string(port) +
          " caps=\"application/x-rtp, media=video, encoding-name=JPEG, payload=26\" ! "
          "rtpjpegdepay ! jpegdec ! videoconvert ! "
          "queue max-size-buffers=1 leaky=downstream ! "
          "appsink drop=true sync=false max-buffers=1";
        RCLCPP_INFO(this->get_logger(), "[OAK-D] Using MJPEG receiver pipeline (fallback)");
      }
    } else {
      // ZED / D435 stream H264-over-RTP
      feed_name = "ZED";
      pipeline = "udpsrc port=" + std::to_string(port) +
        " buffer-size=2147483647 caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264\" ! "
        "rtpjitterbuffer latency=50 ! rtph264depay ! decodebin ! videoconvert ! "
        "appsink drop=true sync=false";
    }
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

      while (running_ && rclcpp::ok()) {
        if (!cap.read(frame) || frame.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }

        {
          std::lock_guard<std::mutex> lock(img_mutex_);
          latest_images_[feed_name] = frame.clone();
        }
        
        frames_received_[feed_name]++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_timers_[feed_name]).count();
        if (elapsed >= 60.0) {
          frames_received_[feed_name] = 0;
          fps_timers_[feed_name] = now;
        }
      }
    }
  }

  void drawTelemetryOverlay(cv::Mat& dashboard) {
    std::string text;
    {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      if (latest_telemetry_.empty()) return;
      // Concatenate all telemetry lines
      for (const auto& [src, msg] : latest_telemetry_) {
        if (!msg.empty()) {
          if (!text.empty()) text += "  |  ";
          text += msg;
        }
      }
    }
    if (text.empty()) return;

    int font = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.8;
    int thickness = 2;
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font, scale, thickness, &baseline);

    int padding = 10;
    cv::Rect bg_rect(0, 0, dashboard.cols, text_size.height + padding * 2);
    
    cv::Mat overlay;
    dashboard.copyTo(overlay);
    cv::rectangle(overlay, bg_rect, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::addWeighted(overlay, 0.6, dashboard, 0.4, 0, dashboard);

    cv::putText(dashboard, text, cv::Point(padding, text_size.height + padding), font, scale, cv::Scalar(0, 255, 255), thickness);
  }

  void displayLoop() {
    std::string window_name = "Multi-Camera UDP Dashboard";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "[GUI] Opened window '%s'", window_name.c_str());

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

        if (!depth.empty() && depth.channels() == 1) {
          static cv::Mat custom_lut;
          if (custom_lut.empty()) {
            custom_lut = cv::Mat(256, 1, CV_8UC3);
            custom_lut.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0); 
            for (int i = 1; i <= 255; ++i) { 
              float x = i / 255.0f;
              if (x < 0.08f) { custom_lut.at<cv::Vec3b>(i, 0) = cv::Vec3b(0, 0, 0); continue; }
              float base_b = 255.0f - (127.0f * x);
              float base_g = 255.0f * (1.0f - x);
              float base_r = 0.0f;
              float diff = x - 0.135f;
              float c = 1.0f - std::pow(diff / 0.02f, 4.0f);
              if (c < 0.0f) c = 0.0f;
              float b = base_b * (1.0f - c) + 0.0f * c;
              float g = base_g * (1.0f - c) + 0.0f * c;
              float r = base_r * (1.0f - c) + 255.0f * c;
              custom_lut.at<cv::Vec3b>(i, 0) = cv::Vec3b(b, g, r);
            }
          }
          cv::applyColorMap(depth, depth, custom_lut);
        }
        if (!ir.empty() && ir.channels() == 1) {
          ir.convertTo(ir, -1, 1.5, 30);
          cv::cvtColor(ir, ir, cv::COLOR_GRAY2BGR);
        }

        int w = 270, h = 480; 
        if (!depth.empty()) { w = depth.cols; h = depth.rows; }
        else if (!ir.empty()) { w = ir.cols; h = ir.rows; }

        cv::Mat blank = cv::Mat::zeros(h, w, CV_8UC3);
        if (depth.empty()) depth = blank.clone();
        if (ir.empty()) ir = blank.clone();

        cv::putText(depth, "Cam " + std::to_string(i) + " Depth", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        cv::putText(ir, "Cam " + std::to_string(i) + " IR", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        cv::Mat cam_grid;
        cv::vconcat(ir, depth, cam_grid);
        camera_grids.push_back(cam_grid);
      }

      cv::Mat zed_frame;
      {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (latest_images_.count("OAK-D")) zed_frame = latest_images_["OAK-D"].clone();
        else if (latest_images_.count("ZED")) zed_frame = latest_images_["ZED"].clone();
      }

      cv::Mat dashboard;
      if (!camera_grids.empty()) {
        cv::Mat final_grid = camera_grids[0];
        for (size_t i = 1; i < camera_grids.size(); ++i) {
          cv::hconcat(final_grid, camera_grids[i], final_grid);
        }

        if (!zed_frame.empty()) {
          double scale = (double)zed_frame.rows / final_grid.rows;
          int new_width = std::round(final_grid.cols * scale);
          cv::Mat final_grid_scaled;
          cv::resize(final_grid, final_grid_scaled, cv::Size(new_width, zed_frame.rows));
          cv::putText(zed_frame, "Primary RGB Feed", cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);
          cv::hconcat(zed_frame, final_grid_scaled, dashboard);
        } else {
          dashboard = final_grid;
        }
      } else if (!zed_frame.empty()) {
        dashboard = zed_frame;
        cv::putText(dashboard, "Primary RGB Feed", cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);
      }

      if (dashboard.empty()) {
        dashboard = cv::Mat::zeros(720, 1280, CV_8UC3);
        cv::putText(dashboard, "Waiting for UDP streams...", cv::Point(60, 120),
                    cv::FONT_HERSHEY_SIMPLEX, 1.6, cv::Scalar(255, 255, 255), 3);
        cv::putText(dashboard, "Open the sender first, then this window will fill automatically.",
                    cv::Point(60, 180), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                    cv::Scalar(180, 180, 180), 2);
        static bool logged_placeholder = false;
        if (!logged_placeholder) {
          RCLCPP_INFO(this->get_logger(), "[GUI] Showing placeholder dashboard until streams arrive");
          logged_placeholder = true;
        }
      }

      cv::resize(dashboard, dashboard, cv::Size(), 1.0, 1.0);
      drawTelemetryOverlay(dashboard);
      static int imshow_counter = 0;
      if (++imshow_counter == 1) {
        RCLCPP_INFO(this->get_logger(), "[GUI] Calling imshow for the first time");
      }
      cv::imshow(window_name, dashboard);

      int key = cv::waitKey(15);
      if (key == 27 || key == 'q') {
        rclcpp::shutdown();
      }
    }
  }

  int num_cameras_;
  std::string rgb_source_;
  std::string oakd_codec_;
  
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub_depth_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub_ir_;

  std::mutex img_mutex_;
  std::map<std::string, cv::Mat> latest_images_;
  std::map<std::string, int> frames_received_;
  std::map<std::string, std::chrono::steady_clock::time_point> fps_timers_;

  std::mutex telemetry_mutex_;
  std::map<std::string, std::string> latest_telemetry_;
  std::string last_oakd_latency_str_;

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