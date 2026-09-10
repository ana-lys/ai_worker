// oakd_apriltag_detector.cpp
//
// AprilTag board-pose detector reading the OAK-D 720p UDP stream directly
// (same feed ffw_stream's realsense_udp_receiver displays) -- no ROS image
// topic in between.
//
// Slice 1 (this file, so far): capture skeleton only. A dedicated capture
// thread decodes the UDP stream via the same GStreamer pipeline
// realsense_udp_receiver.cpp uses for OAK-D 720p (oakd720pStreamLoop,
// copied verbatim below) and keeps ONLY the newest decoded frame
// (drop-oldest, mutex-protected). No AprilTag detection yet -- that is
// wired in a later slice as a second, independent consumer thread that
// snapshots this same buffer without blocking capture.
//
// Usage:
//   oakd_apriltag_detector --ros-args -p oakd_720p_video_port:=9110 -p oakd_codec:=h264
//
// Build: colcon build --packages-select ffw_stream

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

class OakdAprilTagDetector : public rclcpp::Node {
 public:
  OakdAprilTagDetector() : Node("oakd_apriltag_detector") {
    this->declare_parameter<int>("oakd_720p_video_port", 9110);
    this->declare_parameter<std::string>("oakd_codec", "h264");

    port_ = this->get_parameter("oakd_720p_video_port").as_int();
    codec_ = this->get_parameter("oakd_codec").as_string();

    RCLCPP_INFO(this->get_logger(),
                "oakd_apriltag_detector: capture-only (slice 1) -- port=%d codec=%s",
                port_, codec_.c_str());

    capture_thread_ = std::thread(&OakdAprilTagDetector::captureLoop, this);
    report_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&OakdAprilTagDetector::reportStatus, this));
  }

  ~OakdAprilTagDetector() override {
    running_ = false;
    if (capture_thread_.joinable()) capture_thread_.join();
  }

 private:
  // Verbatim from realsense_udp_receiver.cpp's oakd720pStreamLoop() --
  // same low-latency pipelines, so this node decodes the identical feed
  // the dashboard receiver would, just without the ROS Image publish step.
  std::string buildPipeline() const {
    if (codec_ == "mjpeg") {
      return "udpsrc port=" + std::to_string(port_) +
             " buffer-size=2147483647 "
             "caps=\"application/x-rtp, media=video, encoding-name=JPEG, payload=26\" ! "
             "rtpjpegdepay ! jpegdec ! videoconvert ! "
             "queue max-size-buffers=1 leaky=downstream ! "
             "appsink drop=true sync=false async=false max-buffers=1";
    }
    return "udpsrc port=" + std::to_string(port_) +
           " buffer-size=2147483647 "
           "caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264\" ! "
           "rtpjitterbuffer latency=20 ! rtph264depay ! decodebin ! videoconvert ! "
           "queue max-size-buffers=1 leaky=downstream ! "
           "appsink drop=true sync=false async=false max-buffers=1";
  }

  void captureLoop() {
    const std::string pipeline = buildPipeline();
    RCLCPP_INFO(this->get_logger(), "Starting capture on: %s", pipeline.c_str());

    while (running_ && rclcpp::ok()) {
      cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
      if (!cap.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "Failed to open UDP stream, retrying in 2s...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Connected: UDP port %d (codec %s)", port_, codec_.c_str());

      cv::Mat frame;
      while (running_ && rclcpp::ok()) {
        if (!cap.read(frame) || frame.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }

        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          latest_frame_ = frame;  // cv::Mat copy is a cheap header+refcount bump
          frame_generation_++;
        }
        frames_captured_.fetch_add(1, std::memory_order_relaxed);
      }
      // Loop fell through (running_ cleared, or rclcpp shut down) --
      // fall out to the outer while's condition check and exit cleanly.
    }
  }

  // Snapshot the latest frame + its generation counter. is_new tells the
  // caller whether this generation differs from one they already saw --
  // the pattern the detection-thread consumer will use in the next slice.
  bool snapshot(cv::Mat &out, uint64_t &generation) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) return false;
    out = latest_frame_;
    generation = frame_generation_;
    return true;
  }

  void reportStatus() {
    cv::Mat frame;
    uint64_t gen = 0;
    const bool have_frame = snapshot(frame, gen);
    RCLCPP_INFO(this->get_logger(),
                "captured=%lu frames total%s",
                static_cast<unsigned long>(frames_captured_.load()),
                have_frame ? (", latest=" + std::to_string(frame.cols) + "x" +
                              std::to_string(frame.rows) + " gen=" + std::to_string(gen))
                                 .c_str()
                           : ", no frame yet");
  }

  int port_ = 9110;
  std::string codec_ = "h264";

  std::thread capture_thread_;
  std::atomic<bool> running_{true};
  std::atomic<uint64_t> frames_captured_{0};

  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  uint64_t frame_generation_ = 0;

  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OakdAprilTagDetector>());
  rclcpp::shutdown();
  return 0;
}
