#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <algorithm>

rs2_format getFormat(const std::string& fmt) {
    if (fmt == "rgb8") return RS2_FORMAT_RGB8;
    if (fmt == "bgr8") return RS2_FORMAT_BGR8;
    if (fmt == "z16") return RS2_FORMAT_Z16;
    if (fmt == "y8") return RS2_FORMAT_Y8;
    if (fmt == "y16") return RS2_FORMAT_Y16;
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

        this->declare_parameter<int>("target_port_metadata", 8089);
        
        target_ip_ = this->get_parameter("target_ip").as_string();

        metadata_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        memset(&metadata_addr_, 0, sizeof(metadata_addr_));
        metadata_addr_.sin_family = AF_INET;
        metadata_addr_.sin_port = htons(this->get_parameter("target_port_metadata").as_int());
        inet_pton(AF_INET, target_ip_.c_str(), &metadata_addr_.sin_addr);

        discoverDevices();

        // Configure pipeline
        rs2::config cfg;
        std::string device_id = this->get_parameter("device_id").as_string();
        if (!device_id.empty()) {
            RCLCPP_INFO(this->get_logger(), "Requesting specific device_id: %s", device_id.c_str());
            cfg.enable_device(device_id);
        }

        if (this->get_parameter("enable_rgb").as_bool()) {
            int w = this->get_parameter("rgb_width").as_int();
            int h = this->get_parameter("rgb_height").as_int();
            int fps = this->get_parameter("rgb_fps").as_int();
            cfg.enable_stream(RS2_STREAM_COLOR, w, h, getFormat(this->get_parameter("rgb_format").as_string()), fps);
            
            int port = this->get_parameter("target_port_rgb").as_int();
            std::string pipe_str = "appsrc name=src is-live=true do-timestamp=true format=time max-bytes=2000000 ! video/x-raw,format=RGB,width=" + std::to_string(w) + 
                                   ",height=" + std::to_string(h) + ",framerate=" + std::to_string(fps) + "/1 ! "
                                   "videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast ! "
                                   "rtph264pay config-interval=1 ! udpsink host=" + target_ip_ + " port=" + std::to_string(port);
            
            initGstPipeline(pipe_str, pipeline_rgb_, appsrc_rgb_);
        }

        if (this->get_parameter("enable_depth").as_bool()) {
            int w = this->get_parameter("depth_width").as_int();
            int h = this->get_parameter("depth_height").as_int();
            int fps = this->get_parameter("depth_fps").as_int();
            cfg.enable_stream(RS2_STREAM_DEPTH, w, h, getFormat(this->get_parameter("depth_format").as_string()), fps);
            
            int port = this->get_parameter("target_port_depth").as_int();
            // Depth must be lossless / uncompressed RTP to maintain millimeter precision
            // We trick rtpvrawpay by disguising the 16-bit depth (GRAY16_LE) as UYVY, which is also exactly 2 bytes per pixel.
            std::string pipe_str = "appsrc name=src is-live=true do-timestamp=true format=time max-bytes=2000000 ! video/x-raw,format=UYVY,width=" + std::to_string(w) + 
                                   ",height=" + std::to_string(h) + ",framerate=" + std::to_string(fps) + "/1 ! "
                                   "rtpvrawpay ! udpsink host=" + target_ip_ + " port=" + std::to_string(port) + " max-bitrate=250000000";
                                   
            initGstPipeline(pipe_str, pipeline_depth_, appsrc_depth_);
        }

        if (this->get_parameter("enable_ir").as_bool()) {
            int w = this->get_parameter("ir_width").as_int();
            int h = this->get_parameter("ir_height").as_int();
            int fps = this->get_parameter("ir_fps").as_int();
            cfg.enable_stream(RS2_STREAM_INFRARED, w, h, getFormat(this->get_parameter("ir_format").as_string()), fps);
            
            int port = this->get_parameter("target_port_ir").as_int();
            // IR is usually sent as raw grayscale, but we compress it if possible.
            // Wait, D405 IR stride is often 2 bytes per pixel. Let's use GRAY16_LE as the cap, then convert down to 8-bit to compress.
            // "videoconvert ! video/x-raw,format=GRAY8 ! nvvidconv ! ... nvv4l2h264enc"
            std::string pipe_str = "appsrc name=src is-live=true do-timestamp=true format=time max-bytes=2000000 ! video/x-raw,format=GRAY16_LE,width=" + std::to_string(w) + 
                                   ",height=" + std::to_string(h) + ",framerate=" + std::to_string(fps) + "/1 ! "
                                   "videoconvert ! video/x-raw,format=GRAY8 ! "
                                   "videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast ! "
                                   "rtph264pay config-interval=1 ! udpsink host=" + target_ip_ + " port=" + std::to_string(port);
                                   
            initGstPipeline(pipe_str, pipeline_ir_, appsrc_ir_);
        }

        try {
            profile_ = pipe_.start(cfg);
            
            auto dev = profile_.get_device();
            for (auto& sensor : dev.query_sensors()) {
                if (sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
                    sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "RealSense pipeline started successfully. Hardware locked to requested FPS.");
        } catch (const rs2::error & e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense config error: %s", e.what());
            return;
        }

        running_ = true;
        stream_thread_ = std::thread(&RealsenseUDPStreamer::streamLoop, this);
    }

    ~RealsenseUDPStreamer() {
        running_ = false;
        if (stream_thread_.joinable()) {
            stream_thread_.join();
        }
        try {
            pipe_.stop();
        } catch (...) {
        }
        
        if (pipeline_rgb_) { gst_element_set_state(pipeline_rgb_, GST_STATE_NULL); gst_object_unref(pipeline_rgb_); }
        if (pipeline_depth_) { gst_element_set_state(pipeline_depth_, GST_STATE_NULL); gst_object_unref(pipeline_depth_); }
        if (pipeline_ir_) { gst_element_set_state(pipeline_ir_, GST_STATE_NULL); gst_object_unref(pipeline_ir_); }
    }

private:
    void initGstPipeline(const std::string& pipe_str, GstElement*& pipeline, GstElement*& appsrc) {
        GError *error = nullptr;
        pipeline = gst_parse_launch(pipe_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer parse error: %s", error->message);
            g_error_free(error);
            return;
        }
        appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        RCLCPP_INFO(this->get_logger(), "Started GStreamer Pipeline: %s", pipe_str.c_str());
    }

    void discoverDevices() {
        try {
            rs2::context ctx;
            auto devices = ctx.query_devices();
            RCLCPP_INFO(this->get_logger(), "--- RealSense Device Discovery ---");
            RCLCPP_INFO(this->get_logger(), "Found %zu RealSense device(s)", (size_t)devices.size());
            for (size_t i = 0; i < (size_t)devices.size(); ++i) {
                auto dev = devices[i];
                std::string name = dev.supports(RS2_CAMERA_INFO_NAME) ? dev.get_info(RS2_CAMERA_INFO_NAME) : "Unknown";
                std::string serial = dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "Unknown";
                RCLCPP_INFO(this->get_logger(), "  [%zu] %s (Serial: %s)", i, name.c_str(), serial.c_str());
            }
            RCLCPP_INFO(this->get_logger(), "----------------------------------");
        } catch (const rs2::error & e) {
            RCLCPP_WARN(this->get_logger(), "Could not discover devices: %s", e.what());
        }
    }

    void streamLoop() {
        uint32_t frame_id = 0;
        
        bool e_rgb = this->get_parameter("enable_rgb").as_bool();
        bool e_dep = this->get_parameter("enable_depth").as_bool();
        bool e_ir  = this->get_parameter("enable_ir").as_bool();

        auto start_time = std::chrono::steady_clock::now();

        while (running_ && rclcpp::ok()) {
            rs2::frameset frames;
            try {
                frames = pipe_.wait_for_frames(5000);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for frames: %s", e.what());
                continue;
            }

            auto now = std::chrono::system_clock::now();
            double current_timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
            
            if (e_rgb && appsrc_rgb_) pushGstBuffer(frames.get_color_frame(), appsrc_rgb_, "RGB", frame_id, current_timestamp);
            if (e_dep && appsrc_depth_) pushGstBuffer(frames.get_depth_frame(), appsrc_depth_, "Depth", frame_id, current_timestamp);
            if (e_ir && appsrc_ir_) pushGstBuffer(frames.get_infrared_frame(), appsrc_ir_, "IR", frame_id, current_timestamp);
            
            frame_id++;
        }
    }

    void pushGstBuffer(const rs2::video_frame& frame, GstElement* appsrc, const std::string& stream_name, uint32_t frame_id, double timestamp) {
        if (!frame || !appsrc) return;
        
        const uint8_t* data = (const uint8_t*)frame.get_data();
        uint32_t total_size = frame.get_height() * frame.get_stride_in_bytes();
        
        GstBuffer *buffer = gst_buffer_new_allocate(NULL, total_size, NULL);
        gst_buffer_fill(buffer, 0, data, total_size);
        
        // Push buffer
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);
        
        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error pushing buffer to GStreamer");
        } else {
            // Send metadata packet
            char msg[256];
            int len = snprintf(msg, sizeof(msg), "{\"stream\":\"%s\",\"frame_id\":%u,\"timestamp\":%f}", stream_name.c_str(), frame_id, timestamp);
            sendto(metadata_sock_, msg, len, 0, (struct sockaddr*)&metadata_addr_, sizeof(metadata_addr_));
        }
    }

    std::string target_ip_;

    GstElement* pipeline_rgb_ = nullptr;
    GstElement* appsrc_rgb_ = nullptr;
    GstElement* pipeline_depth_ = nullptr;
    GstElement* appsrc_depth_ = nullptr;
    GstElement* pipeline_ir_ = nullptr;
    GstElement* appsrc_ir_ = nullptr;

    rs2::pipeline pipe_;
    rs2::pipeline_profile profile_;
    std::thread stream_thread_;
    std::atomic<bool> running_{false};
    
    int metadata_sock_ = -1;
    struct sockaddr_in metadata_addr_;
};

int main(int argc, char** argv) {
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealsenseUDPStreamer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
