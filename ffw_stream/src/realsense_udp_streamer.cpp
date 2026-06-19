#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

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
    // Network
    this->declare_parameter<std::string>("target_ip", "127.0.0.1");
    this->declare_parameter<int>("target_port_rgb", 8080);
    this->declare_parameter<int>("target_port_depth", 8082);
    this->declare_parameter<int>("target_port_ir", 8084);

    // Device
    this->declare_parameter<std::string>("device_id", "");

    // Streams enabled
    this->declare_parameter<bool>("enable_rgb", true);
    this->declare_parameter<bool>("enable_depth", true);
    this->declare_parameter<bool>("enable_ir", false);

    // Set to false to open the RS2 camera pipeline without starting any GStreamer
    // pipelines. Useful to isolate camera init crashes from GStreamer crashes.
    this->declare_parameter<bool>("enable_gstreamer", true);

    // Stream Configs
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

    // IR stream index (1 = left, 2 = right on stereo modules like D435/D455)
    this->declare_parameter<int>("ir_index", 1);

    this->declare_parameter<int>("target_port_metadata", 8089);

    target_ip_ = this->get_parameter("target_ip").as_string();
    ir_index_ = this->get_parameter("ir_index").as_int();

    metadata_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&metadata_addr_, 0, sizeof(metadata_addr_));
    metadata_addr_.sin_family = AF_INET;
    metadata_addr_.sin_port =
        htons(this->get_parameter("target_port_metadata").as_int());
    inet_pton(AF_INET, target_ip_.c_str(), &metadata_addr_.sin_addr);

    discoverDevices();

    // Configure pipeline
    rs2::config cfg;
    std::string device_id = this->get_parameter("device_id").as_string();
    if (!device_id.empty()) {
      RCLCPP_INFO(this->get_logger(), "Requesting specific device_id: %s",
                  device_id.c_str());
      cfg.enable_device(device_id);
    }

    bool enable_gst = this->get_parameter("enable_gstreamer").as_bool();
    if (!enable_gst) {
      RCLCPP_WARN(this->get_logger(),
                  "GStreamer disabled (enable_gstreamer=false). Camera will open "
                  "but no frames will be streamed over UDP.");
    }

    if (this->get_parameter("enable_rgb").as_bool()) {
      int w = this->get_parameter("rgb_width").as_int();
      int h = this->get_parameter("rgb_height").as_int();
      int fps = this->get_parameter("rgb_fps").as_int();
      cfg.enable_stream(
          RS2_STREAM_COLOR, w, h,
          getFormat(this->get_parameter("rgb_format").as_string()), fps);

      if (enable_gst) {
        int port = this->get_parameter("target_port_rgb").as_int();
        std::string pipe_str =
            "appsrc name=src is-live=true format=time max-bytes=2000000 ! "
            "video/x-raw,format=RGB,width=" +
            std::to_string(w) + ",height=" + std::to_string(h) +
            ",framerate=" + std::to_string(fps) +
            "/1 ! "
            "videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency "
            "speed-preset=ultrafast ! "
            "rtph264pay config-interval=1 ! udpsink host=" +
            target_ip_ + " port=" + std::to_string(port);
        initGstPipeline(pipe_str, pipeline_rgb_, appsrc_rgb_);
      }
    }

    if (this->get_parameter("enable_depth").as_bool()) {
      int w = this->get_parameter("depth_width").as_int();
      int h = this->get_parameter("depth_height").as_int();
      int fps = this->get_parameter("depth_fps").as_int();
      cfg.enable_stream(
          RS2_STREAM_DEPTH, w, h,
          getFormat(this->get_parameter("depth_format").as_string()), fps);

      if (enable_gst) {
        int port = this->get_parameter("target_port_depth").as_int();
        // Depth frames are Z16 (16-bit grayscale), NOT UYVY. Use GRAY16_LE caps
        // so the pipeline is self-describing instead of relying on incidental
        // byte-width overlap with UYVY (which previously "worked" only because
        // both formats are 2 bytes/pixel).
        std::string pipe_str =
            "appsrc name=src is-live=true format=time max-bytes=2000000 ! "
            "video/x-raw,format=GRAY16_LE,width=" +
            std::to_string(w) + ",height=" + std::to_string(h) +
            ",framerate=" + std::to_string(fps) +
            "/1 ! "
            "rtpvrawpay ! udpsink host=" +
            target_ip_ + " port=" + std::to_string(port) +
            " max-bitrate=250000000";
        initGstPipeline(pipe_str, pipeline_depth_, appsrc_depth_);
      }
    }

    if (this->get_parameter("enable_ir").as_bool()) {
      int w = this->get_parameter("ir_width").as_int();
      int h = this->get_parameter("ir_height").as_int();
      int fps = this->get_parameter("ir_fps").as_int();
      cfg.enable_stream(RS2_STREAM_INFRARED, ir_index_, w, h,
                        getFormat(this->get_parameter("ir_format").as_string()),
                        fps);

      if (enable_gst) {
        int port = this->get_parameter("target_port_ir").as_int();
        std::string pipe_str =
            "appsrc name=src is-live=true format=time max-bytes=2000000 ! "
            "video/x-raw,format=GRAY8,width=" +
            std::to_string(w) + ",height=" + std::to_string(h) +
            ",framerate=" + std::to_string(fps) +
            "/1 ! "
            "videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency "
            "speed-preset=ultrafast ! "
            "rtph264pay config-interval=1 ! udpsink host=" +
            target_ip_ + " port=" + std::to_string(port);
        initGstPipeline(pipe_str, pipeline_ir_, appsrc_ir_);
      }
    }

    try {
      profile_ = pipe_.start(cfg);

      auto dev = profile_.get_device();
      for (auto &sensor : dev.query_sensors()) {
        if (sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
          sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f);
        }
        // Query depth_scale from the depth sensor
        if (sensor.is<rs2::depth_sensor>()) {
          depth_scale_ = sensor.as<rs2::depth_sensor>().get_depth_scale();
          RCLCPP_INFO(this->get_logger(),
                      "Depth scale: %f (1 raw unit = %f meters)", depth_scale_,
                      depth_scale_);
        }
      }

      RCLCPP_INFO(this->get_logger(),
                  "RealSense pipeline started successfully. Hardware locked to "
                  "requested FPS.");
    } catch (const rs2::error &e) {
      RCLCPP_ERROR(this->get_logger(), "RealSense config error: %s", e.what());
      return;
    }

    running_ = true;
    stream_thread_rgb_ =
        std::thread(&RealsenseUDPStreamer::streamLoopRgb, this);
    stream_thread_depth_ =
        std::thread(&RealsenseUDPStreamer::streamLoopDepth, this);
    stream_thread_ir_ = std::thread(&RealsenseUDPStreamer::streamLoopIr, this);
  }

  ~RealsenseUDPStreamer() {
    running_ = false;
    if (stream_thread_rgb_.joinable())
      stream_thread_rgb_.join();
    if (stream_thread_depth_.joinable())
      stream_thread_depth_.join();
    if (stream_thread_ir_.joinable())
      stream_thread_ir_.join();

    try {
      pipe_.stop();
    } catch (...) {
    }

    teardownPipeline(pipeline_rgb_, appsrc_rgb_, bus_watch_id_rgb_);
    teardownPipeline(pipeline_depth_, appsrc_depth_, bus_watch_id_depth_);
    teardownPipeline(pipeline_ir_, appsrc_ir_, bus_watch_id_ir_);

    if (main_loop_) {
      g_main_loop_quit(main_loop_);
    }
    if (main_loop_thread_.joinable())
      main_loop_thread_.join();
    if (main_loop_) {
      g_main_loop_unref(main_loop_);
      main_loop_ = nullptr;
    }

    if (metadata_sock_ >= 0) {
      close(metadata_sock_);
      metadata_sock_ = -1;
    }
  }

private:
  void teardownPipeline(GstElement *&pipeline, GstElement *&appsrc,
                        guint &bus_watch_id) {
    if (bus_watch_id != 0) {
      g_source_remove(bus_watch_id);
      bus_watch_id = 0;
    }
    if (appsrc) {
      gst_object_unref(appsrc);
      appsrc = nullptr;
    }
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
      pipeline = nullptr;
    }
  }

  static gboolean busCallback(GstBus * /*bus*/, GstMessage *msg,
                              gpointer user_data) {
    auto *self = static_cast<RealsenseUDPStreamer *>(user_data);
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

  void initGstPipeline(const std::string &pipe_str, GstElement *&pipeline,
                       GstElement *&appsrc) {
    // Initialize GStreamer lazily so that nodes running with enable_gstreamer=false
    // never call gst_init(). On Jetson, gst_init() crashes in headless mode due to
    // NVIDIA EGL GStreamer plugins being mounted by the Docker runtime.
    static bool gst_initialized = false;
    if (!gst_initialized) {
      gst_init(nullptr, nullptr);
      gst_initialized = true;
    }

    GError *error = nullptr;
    pipeline = gst_parse_launch(pipe_str.c_str(), &error);
    if (error) {
      RCLCPP_ERROR(this->get_logger(), "GStreamer parse error: %s",
                   error->message);
      g_error_free(error);
      pipeline = nullptr;
      return;
    }
    if (!pipeline) {
      RCLCPP_ERROR(this->get_logger(),
                   "GStreamer pipeline is NULL after parse (no error reported). "
                   "Check that all required plugins are installed. Pipeline: %s",
                   pipe_str.c_str());
      return;
    }
    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    if (!appsrc) {
      RCLCPP_ERROR(this->get_logger(),
                   "GStreamer appsrc element 'src' not found in pipeline.");
      gst_object_unref(pipeline);
      pipeline = nullptr;
      return;
    }

    ensureMainLoop();
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    guint watch_id = gst_bus_add_watch(bus, busCallback, this);
    gst_object_unref(bus);

    if (pipeline == pipeline_rgb_)
      bus_watch_id_rgb_ = watch_id;
    else if (pipeline == pipeline_depth_)
      bus_watch_id_depth_ = watch_id;
    else if (pipeline == pipeline_ir_)
      bus_watch_id_ir_ = watch_id;

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    RCLCPP_INFO(this->get_logger(), "Started GStreamer Pipeline: %s",
                pipe_str.c_str());
  }

  void discoverDevices() {
    try {
      rs2::context ctx;
      auto devices = ctx.query_devices();
      RCLCPP_INFO(this->get_logger(), "--- RealSense Device Discovery ---");
      RCLCPP_INFO(this->get_logger(), "Found %zu RealSense device(s)",
                  (size_t)devices.size());
      for (size_t i = 0; i < (size_t)devices.size(); ++i) {
        auto dev = devices[i];
        std::string name = dev.supports(RS2_CAMERA_INFO_NAME)
                               ? dev.get_info(RS2_CAMERA_INFO_NAME)
                               : "Unknown";
        std::string serial = dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                 ? dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                 : "Unknown";
        RCLCPP_INFO(this->get_logger(), "  [%zu] %s (Serial: %s)", i,
                    name.c_str(), serial.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "----------------------------------");
    } catch (const rs2::error &e) {
      RCLCPP_WARN(this->get_logger(), "Could not discover devices: %s",
                  e.what());
    }
  }

  // Each enabled stream gets its own thread so a stall/backpressure on one
  // GStreamer pipeline (e.g. depth's udpsink) can't delay delivery of the
  // others. All three pull from the same rs2::pipeline frameset queue
  // independently via poll_for_frames-style waits guarded by their own cadence;
  // we still source frames from one frameset per loop iteration on the
  // depth/rgb/ir-specific wait to keep things simple and correct.
  void streamLoopRgb() {
    if (!(this->get_parameter("enable_rgb").as_bool() && appsrc_rgb_))
      return;
    uint32_t frame_id = 0;
    while (running_ && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        frames = pipe_.wait_for_frames(5000);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "[RGB] Timeout waiting for frames: %s",
                    e.what());
        continue;
      }
      double ts = std::chrono::duration<double>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
      pushGstBuffer(frames.get_color_frame(), appsrc_rgb_, "RGB", frame_id++,
                    ts);
    }
  }

  void streamLoopDepth() {
    if (!(this->get_parameter("enable_depth").as_bool() && appsrc_depth_))
      return;
    uint32_t frame_id = 0;
    while (running_ && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        frames = pipe_.wait_for_frames(5000);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "[Depth] Timeout waiting for frames: %s", e.what());
        continue;
      }
      double ts = std::chrono::duration<double>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
      pushGstBuffer(frames.get_depth_frame(), appsrc_depth_, "Depth",
                    frame_id++, ts);
    }
  }

  void streamLoopIr() {
    if (!(this->get_parameter("enable_ir").as_bool() && appsrc_ir_))
      return;
    uint32_t frame_id = 0;
    while (running_ && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        frames = pipe_.wait_for_frames(5000);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "[IR] Timeout waiting for frames: %s",
                    e.what());
        continue;
      }
      double ts = std::chrono::duration<double>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
      pushGstBuffer(frames.get_infrared_frame(ir_index_), appsrc_ir_, "IR",
                    frame_id++, ts);
    }
  }

  void pushGstBuffer(const rs2::video_frame &frame, GstElement *appsrc,
                     const std::string &stream_name, uint32_t frame_id,
                     double timestamp) {
    if (!frame || !appsrc)
      return;

    const uint8_t *data = (const uint8_t *)frame.get_data();

    // Sanity-check stride: if the SDK ever returns padded rows, width-based
    // caps downstream would misinterpret row boundaries. Warn once per stream
    // if seen.
    uint32_t expected_stride = frame.get_width() * frame.get_bytes_per_pixel();
    if (frame.get_stride_in_bytes() != (int)expected_stride) {
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "%s: stride (%d) != width*bpp (%u) -- frame has row "
                       "padding, downstream caps may misinterpret data!",
                       stream_name.c_str(), frame.get_stride_in_bytes(),
                       expected_stride);
    }

    uint32_t total_size = frame.get_height() * frame.get_stride_in_bytes();

    GstBuffer *buffer = gst_buffer_new_allocate(NULL, total_size, NULL);
    gst_buffer_fill(buffer, 0, data, total_size);

    // Explicit PTS based on the pipeline clock instead of relying solely on
    // do-timestamp, which only stamps buffers as they enter appsrc and can
    // behave inconsistently with downstream elements (encoders, jitterbuffers,
    // queues with max-size-time) that expect a coherent running clock.
    GstClock *clock = gst_element_get_clock(appsrc);
    if (clock) {
      GstClockTime now = gst_clock_get_time(clock);
      GstClockTime base = gst_element_get_base_time(appsrc);
      GST_BUFFER_PTS(buffer) = (now > base) ? (now - base) : 0;
      gst_object_unref(clock);
    }

    // Push buffer
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Error pushing buffer to GStreamer");
    } else {
      // Send metadata packet (includes depth_scale for proper Z16
      // interpretation)
      char msg[256];
      int len =
          snprintf(msg, sizeof(msg),
                   "{\"stream\":\"%s\",\"frame_id\":%u,\"timestamp\":%f,"
                   "\"depth_scale\":%e}",
                   stream_name.c_str(), frame_id, timestamp, depth_scale_);
      sendto(metadata_sock_, msg, len, 0, (struct sockaddr *)&metadata_addr_,
             sizeof(metadata_addr_));
    }
  }

  std::string target_ip_;
  int ir_index_ = 1;

  GstElement *pipeline_rgb_ = nullptr;
  GstElement *appsrc_rgb_ = nullptr;
  GstElement *pipeline_depth_ = nullptr;
  GstElement *appsrc_depth_ = nullptr;
  GstElement *pipeline_ir_ = nullptr;
  GstElement *appsrc_ir_ = nullptr;

  guint bus_watch_id_rgb_ = 0;
  guint bus_watch_id_depth_ = 0;
  guint bus_watch_id_ir_ = 0;
  GMainLoop *main_loop_ = nullptr;
  std::thread main_loop_thread_;

  rs2::pipeline pipe_;
  rs2::pipeline_profile profile_;
  std::thread stream_thread_rgb_;
  std::thread stream_thread_depth_;
  std::thread stream_thread_ir_;
  std::atomic<bool> running_{false};

  int metadata_sock_ = -1;
  struct sockaddr_in metadata_addr_;
  float depth_scale_ = 0.001f; // default, overridden from device at startup
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseUDPStreamer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}