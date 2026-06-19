#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <lz4.h>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

class RealsenseUDPReceiver : public rclcpp::Node {
public:
  RealsenseUDPReceiver() : Node("realsense_udp_receiver") {
    // Ports
    this->declare_parameter<int>("target_port_rgb", 8080);
    this->declare_parameter<int>("target_port_depth", 8082);
    this->declare_parameter<int>("target_port_ir", 8084);
    this->declare_parameter<int>("target_port_metadata", 8089);

    // Camera identity for display namespaces
    this->declare_parameter<std::string>("camera_name", "Camera");
    camera_name_ = this->get_parameter("camera_name").as_string();

    this->declare_parameter<bool>("headless", false);

    // Streams enabled
    this->declare_parameter<bool>("enable_rgb", true);
    this->declare_parameter<bool>("enable_depth", true);
    this->declare_parameter<bool>("enable_ir", false);

    // Stream Configs
    this->declare_parameter<int>("rgb_width", 640);
    this->declare_parameter<int>("rgb_height", 480);
    this->declare_parameter<int>("depth_width", 640);
    this->declare_parameter<int>("depth_height", 480);
    this->declare_parameter<int>("ir_width", 640);
    this->declare_parameter<int>("ir_height", 480);

    // Publishers
    pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/color/image_raw", 10);
    pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/depth/image_rect_raw", 10);
    pub_ir_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/infra1/image_rect_raw", 10);

    if (this->get_parameter("enable_rgb").as_bool()) {
      int port = this->get_parameter("target_port_rgb").as_int();
      std::string pipe_str =
          "udpsrc port=" + std::to_string(port) +
          " ! "
          "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
          "rtph264depay ! decodebin ! videoconvert ! video/x-raw,format=BGR ! "
          "appsink name=sink drop=true max-buffers=1 sync=false";
      initGstPipeline("RGB", pipe_str, pipeline_rgb_, appsink_rgb_,
                      bus_watch_id_rgb_);
    }

    if (this->get_parameter("enable_depth").as_bool()) {
      int port = this->get_parameter("target_port_depth").as_int();
      std::string pipe_str =
          "udpsrc port=" + std::to_string(port) +
          " ! "
          "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
          "rtph264depay ! decodebin ! videoconvert ! video/x-raw,format=GRAY8 "
          "! appsink name=sink drop=true max-buffers=1 sync=false";
      initGstPipeline("Depth", pipe_str, pipeline_depth_, appsink_depth_,
                      bus_watch_id_depth_);
    }

    if (this->get_parameter("enable_ir").as_bool()) {
      int port = this->get_parameter("target_port_ir").as_int();
      std::string pipe_str =
          "udpsrc port=" + std::to_string(port) +
          " ! "
          "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
          "rtph264depay ! decodebin ! videoconvert ! video/x-raw,format=GRAY8 "
          "! appsink name=sink drop=true max-buffers=1 sync=false";
      initGstPipeline("IR", pipe_str, pipeline_ir_, appsink_ir_,
                      bus_watch_id_ir_);
    }

    RCLCPP_INFO(this->get_logger(), "C++ UDP Receiver Initialized.");

    metadata_thread_ = std::thread(&RealsenseUDPReceiver::metadataLoop, this);

    if (!this->get_parameter("headless").as_bool()) {
      display_thread_ = std::thread(&RealsenseUDPReceiver::displayLoop, this);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Running in HEADLESS mode (No OpenCV Windows)");
    }
  }

  ~RealsenseUDPReceiver() {
    running_ = false;
    if (display_thread_.joinable())
      display_thread_.join();
    if (metadata_thread_.joinable())
      metadata_thread_.join();

    teardownPipeline(pipeline_rgb_, appsink_rgb_, bus_watch_id_rgb_,
                     callback_data_rgb_);
    teardownPipeline(pipeline_depth_, appsink_depth_, bus_watch_id_depth_,
                     callback_data_depth_);
    teardownPipeline(pipeline_ir_, appsink_ir_, bus_watch_id_ir_,
                     callback_data_ir_);

    if (main_loop_) {
      g_main_loop_quit(main_loop_);
    }
    if (main_loop_thread_.joinable())
      main_loop_thread_.join();
    if (main_loop_) {
      g_main_loop_unref(main_loop_);
      main_loop_ = nullptr;
    }

    cv::destroyAllWindows();
  }

private:
  using CallbackData = std::pair<RealsenseUDPReceiver *, std::string>;

  void teardownPipeline(GstElement *&pipeline, GstElement *&appsink,
                        guint &bus_watch_id, CallbackData *&cb_data) {
    if (bus_watch_id != 0) {
      g_source_remove(bus_watch_id);
      bus_watch_id = 0;
    }
    if (appsink) {
      gst_object_unref(appsink);
      appsink = nullptr;
    }
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
      pipeline = nullptr;
    }
    if (cb_data) {
      delete cb_data;
      cb_data = nullptr;
    }
  }

  static gboolean busCallback(GstBus * /*bus*/, GstMessage *msg,
                              gpointer user_data) {
    auto *self = static_cast<RealsenseUDPReceiver *>(user_data);
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      GError *err = nullptr;
      gchar *dbg = nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      RCLCPP_ERROR(self->get_logger(), "GStreamer ERROR from %s: %s (%s)",
                   GST_OBJECT_NAME(msg->src), err ? err->message : "unknown",
                   dbg ? dbg : "no debug info");
      if (err)
        g_error_free(err);
      if (dbg)
        g_free(dbg);
      break;
    }
    case GST_MESSAGE_WARNING: {
      GError *err = nullptr;
      gchar *dbg = nullptr;
      gst_message_parse_warning(msg, &err, &dbg);
      RCLCPP_WARN(self->get_logger(), "GStreamer WARNING from %s: %s (%s)",
                  GST_OBJECT_NAME(msg->src), err ? err->message : "unknown",
                  dbg ? dbg : "no debug info");
      if (err)
        g_error_free(err);
      if (dbg)
        g_free(dbg);
      break;
    }
    case GST_MESSAGE_EOS:
      RCLCPP_WARN(self->get_logger(), "GStreamer EOS from %s",
                  GST_OBJECT_NAME(msg->src));
      break;
    default:
      break;
    }
    return TRUE;
  }

  void ensureMainLoop() {
    if (main_loop_)
      return;
    main_loop_ = g_main_loop_new(nullptr, FALSE);
    main_loop_thread_ = std::thread([this]() { g_main_loop_run(main_loop_); });
  }

  void initGstPipeline(const std::string &name, const std::string &pipe_str,
                       GstElement *&pipeline, GstElement *&appsink,
                       guint &bus_watch_id) {
    GError *error = nullptr;
    pipeline = gst_parse_launch(pipe_str.c_str(), &error);
    if (error) {
      RCLCPP_ERROR(this->get_logger(), "GStreamer parse error for %s: %s",
                   name.c_str(), error->message);
      g_error_free(error);
      return;
    }

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    gst_app_sink_set_emit_signals(GST_APP_SINK(appsink), true);

    // Pass instance and name. Ownership is tracked so we can free it in
    // teardown.
    auto *data = new CallbackData(this, name);
    if (name == "RGB")
      callback_data_rgb_ = data;
    else if (name == "Depth")
      callback_data_depth_ = data;
    else if (name == "IR")
      callback_data_ir_ = data;
    g_signal_connect(appsink, "new-sample", G_CALLBACK(onNewSample), data);

    ensureMainLoop();
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_watch_id = gst_bus_add_watch(bus, busCallback, this);
    gst_object_unref(bus);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    RCLCPP_INFO(this->get_logger(), "Started %s Pipeline: %s", name.c_str(),
                pipe_str.c_str());
  }

  static GstFlowReturn onNewSample(GstElement *sink, gpointer user_data) {
    auto *data = static_cast<CallbackData *>(user_data);
    data->first->processSample(sink, data->second);
    return GST_FLOW_OK;
  }

  void processSample(GstElement *sink, const std::string &stream_name) {
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample)
      return;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *s = gst_caps_get_structure(caps, 0);

    int w, h;
    gst_structure_get_int(s, "width", &w);
    gst_structure_get_int(s, "height", &h);

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return;
    }

    cv::Mat frame;
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_link";

    // Guard against undersized buffers (e.g. partial/corrupt UDP packets)
    // before wrapping them in a cv::Mat of the expected size, which would
    // otherwise read OOB.
    size_t expected_bytes = 0;
    if (stream_name == "RGB")
      expected_bytes = (size_t)w * h * 3;
    else if (stream_name == "IR")
      expected_bytes = (size_t)w * h * 1;
    else if (stream_name == "Depth")
      expected_bytes = (size_t)w * h * 1; // Since we forced it to GRAY8

    if (map.size < expected_bytes) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[%s] Buffer too small (%zu < %zu), dropping frame",
                           stream_name.c_str(), map.size, expected_bytes);
      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);
      return;
    }

    // depth_scale_ is updated asynchronously by metadataLoop
    float scale;
    {
      std::lock_guard<std::mutex> lock(meta_mutex_);
      scale = depth_scale_;
    }

    if (stream_name == "RGB") {
      frame = cv::Mat(h, w, CV_8UC3, map.data).clone();
      sensor_msgs::msg::Image::SharedPtr msg =
          cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      pub_rgb_->publish(*msg);
    } else if (stream_name == "IR") {
      frame = cv::Mat(h, w, CV_8UC1, map.data).clone();
      sensor_msgs::msg::Image::SharedPtr msg =
          cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
      pub_ir_->publish(*msg);
    } else if (stream_name == "Depth") {
      cv::Mat gray8(h, w, CV_8UC1, map.data);
      frame = cv::Mat(h, w, CV_16UC1);
      
      if (scale <= 0.000001f) scale = 0.001f; // fallback to 1mm if metadata hasn't arrived yet
      float recovery_factor = 1.0f / (255.0f * scale);
      
      for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
          frame.at<uint16_t>(r, c) = (uint16_t)(gray8.at<uint8_t>(r, c) * recovery_factor);
        }
      }
      
      sensor_msgs::msg::Image::SharedPtr msg =
          cv_bridge::CvImage(header, "mono16", frame).toImageMsg();
      pub_depth_->publish(*msg);
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    if (frame.empty())
      return;

    // Save for display loop
    {
      std::lock_guard<std::mutex> lock(img_mutex_);
      latest_images_[stream_name] = frame;
    }

    frames_received_[stream_name]++;
    auto now = std::chrono::system_clock::now();

    double latency = 0.0;
    uint32_t metadata_drops = 0;
    uint32_t video_drops = 0;
    {
      std::lock_guard<std::mutex> lock(meta_mutex_);
      if (latest_timestamp_[stream_name] > 0.0) {
        double current_time =
            std::chrono::duration<double>(now.time_since_epoch()).count();
        latency = current_time - latest_timestamp_[stream_name];
      }
      metadata_drops = total_drops_[stream_name];

      // Calculate video frames lost since last successful metadata received
      if (first_frame_id_[stream_name] == 0 &&
          sender_frame_ids_[stream_name] > 0) {
        first_frame_id_[stream_name] = sender_frame_ids_[stream_name];
      }

      if (first_frame_id_[stream_name] > 0 &&
          sender_frame_ids_[stream_name] >= first_frame_id_[stream_name]) {
        uint32_t expected_frames =
            (sender_frame_ids_[stream_name] - first_frame_id_[stream_name]) + 1;
        if (expected_frames > total_video_frames_received_[stream_name]) {
          video_drops =
              expected_frames - total_video_frames_received_[stream_name];
        }
      }
    }
    total_video_frames_received_[stream_name]++;

    if (frames_received_[stream_name] == 30) {
      auto steady_now = std::chrono::steady_clock::now();
      double elapsed =
          std::chrono::duration<double>(steady_now - fps_timers_[stream_name])
              .count();
      double fps_val = (elapsed > 0.0) ? (30.0 / elapsed) : 0.0;
      RCLCPP_INFO(this->get_logger(),
                  "[%s] Receiver FPS: %.1f | Latency: %.1f ms | Metadata "
                  "Drops: %u | Video Drops: %u",
                  stream_name.c_str(), fps_val, latency * 1000.0,
                  metadata_drops, video_drops);
      frames_received_[stream_name] = 0;
      fps_timers_[stream_name] = steady_now;
    }
  }

  void displayLoop() {
    while (running_ && rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(img_mutex_);
        for (auto &pair : latest_images_) {
          if (!pair.second.empty()) {
            cv::Mat display_img = pair.second;

            if (pair.first == "Depth") {
              // Use depth_scale from metadata for proper conversion
              // depth_scale converts raw Z16 units to meters
              float scale;
              {
                std::lock_guard<std::mutex> mlock(meta_mutex_);
                scale = depth_scale_;
              }
              cv::Mat depth_meters;
              display_img.convertTo(depth_meters, CV_32F, scale);
              // Clamp to 1.0 meter max for visualization
              cv::threshold(depth_meters, depth_meters, 1.0, 1.0,
                            cv::THRESH_TRUNC);
              cv::Mat img_normalized = depth_meters / 1.0 * 255.0;
              img_normalized.convertTo(img_normalized, CV_8UC1);
              cv::applyColorMap(img_normalized, display_img, cv::COLORMAP_JET);
            }

            std::string window_name = camera_name_ + " - " + pair.first;
            cv::imshow(window_name, display_img);
            pair.second = cv::Mat(); // consume
          }
        }
      }
      int key = cv::waitKey(1);
      if (key == 27 || key == 'q') {
        rclcpp::shutdown();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
        buffer[len] = '\0';
        char stream_name[32];
        uint32_t frame_id;
        double timestamp;
        float depth_scale = 0.0f;

        // Try 4-field format (with depth_scale) first, fall back to 3-field
        int parsed = sscanf(buffer,
                            "{\"stream\":\"%31[^\"]\",\"frame_id\":%u,"
                            "\"timestamp\":%lf,\"depth_scale\":%f}",
                            stream_name, &frame_id, &timestamp, &depth_scale);
        if (parsed < 3) {
          parsed = sscanf(
              buffer,
              "{\"stream\":\"%31[^\"]\",\"frame_id\":%u,\"timestamp\":%lf}",
              stream_name, &frame_id, &timestamp);
        }

        if (parsed >= 3) {
          std::string stream(stream_name);
          std::lock_guard<std::mutex> lock(meta_mutex_);

          if (depth_scale > 0.0f) {
            depth_scale_ = depth_scale;
          }

          if (sender_frame_ids_[stream] > 0 &&
              frame_id >= sender_frame_ids_[stream]) {
            uint32_t expected = sender_frame_ids_[stream] + 1;
            if (frame_id > expected) {
              uint32_t missed = frame_id - expected;
              total_drops_[stream] += missed;
              RCLCPP_WARN(this->get_logger(),
                          "[%s] Network Drop Detected! Missed %u frames. Total "
                          "Drops: %u",
                          stream.c_str(), missed, total_drops_[stream]);
            }
          }
          sender_frame_ids_[stream] = frame_id;
          latest_timestamp_[stream] = timestamp;
        }
      }
    }
    close(sock);
  }

  std::string camera_name_;
  GstElement *pipeline_rgb_ = nullptr;
  GstElement *appsink_rgb_ = nullptr;
  GstElement *pipeline_depth_ = nullptr;
  GstElement *appsink_depth_ = nullptr;
  GstElement *pipeline_ir_ = nullptr;
  GstElement *appsink_ir_ = nullptr;

  guint bus_watch_id_rgb_ = 0;
  guint bus_watch_id_depth_ = 0;
  guint bus_watch_id_ir_ = 0;
  GMainLoop *main_loop_ = nullptr;
  std::thread main_loop_thread_;

  CallbackData *callback_data_rgb_ = nullptr;
  CallbackData *callback_data_depth_ = nullptr;
  CallbackData *callback_data_ir_ = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_;

  std::mutex img_mutex_;
  std::map<std::string, cv::Mat> latest_images_;
  std::map<std::string, int> frames_received_;
  std::map<std::string, std::chrono::steady_clock::time_point> fps_timers_;

  std::mutex meta_mutex_;
  std::map<std::string, uint32_t> first_frame_id_;
  std::map<std::string, uint32_t> sender_frame_ids_;
  std::map<std::string, uint32_t> total_drops_;
  std::map<std::string, uint32_t> total_video_frames_received_;
  std::map<std::string, double> latest_timestamp_;
  float depth_scale_ = 0.001f; // default, updated from streamer metadata

  std::thread display_thread_;
  std::thread metadata_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char **argv) {
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseUDPReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}